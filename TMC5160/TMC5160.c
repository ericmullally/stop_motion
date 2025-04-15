/*
 * TMC5160.c
 *
 *  Created on: Mar 19, 2025
 *      Author: Eric
 */


#include "TMC5160.h"
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>


#define PWM_FREQUENCY 	24000
#define MOTOR_VOLTAGE 	10
#define COIL_RESISTANCE 1.6
#define FCLCK			12000000

/* Current Setting:
	IRMS = (GLOBALSCALER \ 256) * (CS+1 \ 32) * (VFS / RSENSE) * (1/ sqrt(2))
	GLOBALSCALER = (IRMS * 256 * 32 * RSENSE * sqrt(2)) /  (CS+1) * VFS
 * */

#define SENSE_RESISTOR  0.075
#define SENSE_VOLTAGE 	0.325
#define SQRT_2			1.41
#define CURRENT_SCALE	31



static const size_t reg_access_count = sizeof(reg_access) / sizeof(reg_access[0]);
uint32_t default_max_velocity = 1600; // usteps/s
uint32_t default_acceleration = 2000; // usteps/s^2

uint32_t current_max_velocity;
uint32_t current_acceleration;


typedef enum{
	TMC5160_SUCCESS 		= 0x00,
	NULL_MOTOR_ERROR 		= 0x01,
	REGISTER_ACCESS_ERROR 	= 0x02,
	COMMUNICATION_ERROR 	= 0x03,
	DRIVER_ERROR			= 0x04,
	STALL_ERROR				= 0x05,
	UNDERVOLTAGE_ERROR		= 0x06,
	POSITION_NOT_REACHED	= 0x07,
	MOTOR_IN_USE			= 0x08
}TMC5160_return_values_t;

typedef enum{
	FORWARD,
	REVERSE
}TMC5160_directions_t;


static void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state);
static void rtos_delay(uint32_t delay_ms);
static uint8_t is_readable(TMC5160_reg_addresses reg_addr);
static TMC5160_return_values_t TMC5160_reset_status(TMC5160_HandleTypeDef *motor);
static uint8_t calculate_globalScaler(int desired_rms_current);
TMC5160_return_values_t move_for_time(TMC5160_HandleTypeDef *motor, uint32_t time_ms, TMC5160_directions_t direction);
TMC5160_return_values_t TMC5160_wait_for_position(TMC5160_HandleTypeDef *motor, int position);
uint16_t tmc_mres_to_microsteps(uint8_t mres);




/*
 * @Brief Write initial motor settings.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
HAL_StatusTypeDef TMC5160_init(TMC5160_HandleTypeDef *motor){

	SPI_Status_t spi_res = TMC5160_ReadRegister(motor, &motor->registers.GSTAT.Val.Value, GSTAT);

	// Disable motor for setup
	TMC5160_motor_enable(motor, GPIO_PIN_SET);

	// Reset flags on start
	(void)TMC5160_reset_status(motor);

	// GLOBAL_SCALER
	motor->registers.GLOBAL_SCALER.Val.BitField.GLOBAL_SCALER = calculate_globalScaler(2);
	(void)TMC5160_WriteRegister(motor, GLOBALSCALER, motor->registers.GLOBAL_SCALER.Val.Value);

	// GCONF
	spi_res = TMC5160_ReadRegister(motor, &motor->registers.GCONF.Val.Value, GCONF);
	motor->registers.GCONF.Val.BitField.EN_PWM_MODE = 1;
	(void)TMC5160_WriteRegister(motor, GCONF, motor->registers.GCONF.Val.Value);

	// CHOPCONF
	spi_res = TMC5160_ReadRegister(motor, &motor->registers.CHOPCONF.Val.Value, CHOPCONF);
	motor->registers.CHOPCONF.Val.BitField.MRES  = 4; //16
	motor->registers.CHOPCONF.Val.BitField.TBL   = 1;
	motor->registers.CHOPCONF.Val.BitField.TOFF  = 4;
	motor->registers.CHOPCONF.Val.BitField.HEND  = 6;
	motor->registers.CHOPCONF.Val.BitField.HSTRT = 0;
	(void)TMC5160_WriteRegister(motor, CHOPCONF, motor->registers.CHOPCONF.Val.Value);

	// TPWMTHRS
	motor->registers.TPWMTHRS.Val.BitField.TPWMTHRS = 0; // always cool step?
	(void)TMC5160_WriteRegister(motor, TPWMTHRS, motor->registers.TPWMTHRS.Val.Value );

	// PWMCONF
	motor->registers.PWMCONF.Val.BitField.PWM_AUTOSCALE = 1;
	motor->registers.PWMCONF.Val.BitField.PWM_AUTOGRAD = 1;
	motor->registers.PWMCONF.Val.BitField.PWM_FREQ = 0; // 24Khz
	motor->registers.PWMCONF.Val.BitField.FREEWHEEL = 0;
	(void)TMC5160_WriteRegister(motor, PWMCONF, motor->registers.PWMCONF.Val.Value );

	// IHOLD_IRUN
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLD = 16;
	motor->registers.IHOLD_IRUN.Val.BitField.IRUN = CURRENT_SCALE;
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLDDELAY = 3;
	(void)TMC5160_WriteRegister(motor, IHOLD_IRUN, motor->registers.IHOLD_IRUN.Val.Value );

	// TPOWERDOWN
	motor->registers.TPOWERDOWN.Val.BitField.TPOWERDOWN = 3;
	(void)TMC5160_WriteRegister(motor, TPOWERDOWN, motor->registers.TPOWERDOWN.Val.Value );

	// RAMP MODE
	motor->registers.RAMPMODE.Val.BitField.RAMPMODE = 0; // position mode
	(void)TMC5160_WriteRegister(motor, RAMPMODE, motor->registers.RAMPMODE.Val.Value);

	TMC5160_set_max_acceleration(motor, default_acceleration);
	TMC5160_set_max_velocity(motor, default_max_velocity);

	return HAL_OK;
}

/*
 * @Brief Tune StealthChop for quiet motor movement.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor){

	// enable the motor
	// wait 100 us
	// perform homing

	TMC5160_motor_enable(motor, GPIO_PIN_RESET);
	rtos_delay(100);

	return HAL_OK;
}

/*
 @Brief Writes data to the desired register.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to write to.
 @Param *data: Pointer to the data to be written, should be a 4 byte array.
 @Ret HAL_StatusTypeDef
 * */
HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data){
	// TODO: this needs to return a more descriptive error using TMC5160_return_values_t

	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		return HAL_ERROR;
	}
	while(motor->spi->State != HAL_SPI_STATE_READY){rtos_delay(1);}

	uint8_t tx_data[5];
	tx_data[0] = (reg_addr | WRITE_MASK);
	tx_data[1] = (uint8_t)(data >> 24 );
	tx_data[2] = (uint8_t)(data >> 16 );
	tx_data[3] = (uint8_t)(data >> 8 );
	tx_data[4] = (uint8_t)(data);

	HAL_StatusTypeDef res =  HAL_SPI_Transmit(motor->spi, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	rtos_delay(1);
	return res;

}

/*
 @Brief reads the requested register and returns the value.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to read.
 @Ret uint32_t: Either the register value or 0xFFFFFFFF indicating an error.
 * */
SPI_Status_t TMC5160_ReadRegister(TMC5160_HandleTypeDef *motor, uint32_t * reg_val, uint8_t reg_addr){

	SPI_Status_t spi_status = {0};

	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		spi_status.Val.Value = 0xFF;
		return spi_status;
	}

	if(!is_readable(reg_addr)){
		spi_status.Val.Value = 0xFF;
		return spi_status;
	}


	uint8_t rx_data[5];
	uint8_t tx_data[5] = {reg_addr, 0x00, 0x00, 0x00, 0x00};

	// first response can be disposed of
	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		spi_status.Val.Value = 0xFF;
		return spi_status;
	}

	memset(rx_data, 0, sizeof(rx_data) );

	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		spi_status.Val.Value = 0xFF;
		return spi_status;
	}else{
		*reg_val = ( (rx_data[1] << 24) | (rx_data[2] << 16)| (rx_data[3] << 8) | rx_data[4] );
		spi_status.Val.Value = rx_data[0];
		return spi_status;
	}

}

/*
 * @Brief Changes the motor's travel speed.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param velocity: new maximum travel velocity in usteps/s
 * */
void TMC5160_set_max_velocity(TMC5160_HandleTypeDef *motor, int velocity){
	/*     µstep velocity µsteps / s:
	  			 VMAX_LIMIT = 8388096 in the register
				 v[Hz] = v[TMC] * ( fCLK[Hz]/ 2^24 )
				 v[TMC] = ( v[Hz] * 2^24 ) / fCLK
	 	 	 	 velocity of 1Hz is 3200 = 200 full steps for 1 rotation * 16usteps
	*/

	int const MAX_VELOCITY = 5999633; // user max requestable value

	if(velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
	if(velocity < 0) velocity = 0;
	current_max_velocity  = velocity;  // We need to know the velocity in us/s motor struct just holds the register value

	uint64_t numerator = ((uint64_t)velocity * (1 << 24)); // max value can surpass 32 bits
	int velocity_max = (int)( numerator / FCLCK );

	motor->registers.VMAX.Val.BitField.VMAX = velocity_max;
	(void)TMC5160_WriteRegister(motor, VMAX, motor->registers.VMAX.Val.Value);

}

/*
 * @Brief Sets the maximum acceleration to achieve the max velocity.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param acceleration: The desired acceleration in usteps/s^2
 * */
void TMC5160_set_max_acceleration(TMC5160_HandleTypeDef *motor, int acceleration){
	//	µstep acceleration a[Hz/s]:
	//	AMAX register = 2^16
	// 		MAX_ACCELERATION = 4291534 for the user
	//		a[Hz/s] = a[TMC] * fCLK[Hz]^2 / ((512*256) * 2^24)
	//		a[TMC] = a[Hz/s]  * (( 512 * 256) * 2^24) / fCLK[Hz]^2
	//		40 = 2000 * (( 512 * 256) * 2^24 / 12MHz^2
	//		40 = 2000 Hz

	int const MAX_ACCELERATION = 4291534; // user max requestable value 2^16
	if(acceleration > MAX_ACCELERATION ) acceleration = MAX_ACCELERATION;
	if(acceleration < 0 ) acceleration = 0;

	// these magic numbers are from the data sheet.
	uint64_t numerator = (uint64_t)acceleration * (512 * 256) * (1<< 24);
	uint64_t denominator = ((uint64_t)FCLCK * FCLCK);
	int acc_max  = (int)(numerator / denominator);

	motor->registers.AMAX.Val.BitField.AMAX = acc_max;
	(void)TMC5160_WriteRegister(motor, AMAX, motor->registers.AMAX.Val.Value);

}

/*
 @Brief Moves the motor at the current velocity for a requested time.
 @Param motor : pointer to the motor struct
 @Param time_ms : the time you want to move.
 @Param direction: The direction to rotate the motor.
 @Note This function will leave the motor in velocity mode.
 * */
TMC5160_return_values_t move_for_time(TMC5160_HandleTypeDef *motor, uint32_t time_ms, TMC5160_directions_t direction){

	SPI_Status_t status = TMC5160_ReadRegister(motor, &motor->registers.GSTAT.Val.Value, GSTAT);

	if(!status.Val.BitField.STANDSTILL){
		return MOTOR_IN_USE;
	}

	// Ensure motor doesn't move until ready.
	TMC5160_set_max_velocity(motor, 0);

	// set velocity mode
	if(motor->registers.RAMPMODE.Val.BitField.RAMPMODE == 0){

		// Set the direction and velocity mode on.
		if(direction == FORWARD){
			motor->registers.RAMPMODE.Val.BitField.RAMPMODE = 1;
		}else{
			motor->registers.RAMPMODE.Val.BitField.RAMPMODE = 2;
		}

	    if(TMC5160_WriteRegister(motor, RAMPMODE, motor->registers.RAMPMODE.Val.Value) != HAL_OK){
	    	return COMMUNICATION_ERROR;
	    }

		// if the motor is disabled we enable it. It wont move until the VMAX is set
		if(HAL_GPIO_ReadPin(motor->motor_en_port, motor->motor_en_pin) != GPIO_PIN_RESET){
			TMC5160_motor_enable(motor, GPIO_PIN_RESET);
			rtos_delay(10);
		}

	}

	// Move the motor at the default velocity
	TMC5160_set_max_velocity(motor, default_max_velocity);

	// Keep going until finished TODO: This needs a stall guard or sensor feedback. we may hit the end of the machine in production
	rtos_delay(time_ms);

	// Stop the motor when finished
	TMC5160_set_max_velocity(motor, 0);
}

/*
 @Brief Moves the motor the number of full steps requested. Forward or reverse is determined but the sign
 	 	of the requested number.
 @Param motor : Pointer to the motor struct.
 @Param full_steps : Positive or negative number of steps to move.
 * */
void TMC5160_move_to_position(TMC5160_HandleTypeDef *motor, int full_steps){

	// Position mode
	if(motor->registers.RAMPMODE.Val.BitField.RAMPMODE != 0){
		motor->registers.RAMPMODE.Val.BitField.RAMPMODE = 0;
	    (void)TMC5160_WriteRegister(motor, RAMPMODE, motor->registers.RAMPMODE.Val.Value);
	}


	uint16_t microsteps =  tmc_mres_to_microsteps((uint8_t)motor->registers.CHOPCONF.Val.BitField.MRES);
	int total_steps = microsteps * full_steps;

	// The sign of full_steps controls direction
	int target_position = motor->registers.XACTUAL.Val.BitField.XACTUAL + total_steps;
	motor->registers.XTARGET.Val.BitField.XTARGET = target_position;
	(void)TMC5160_WriteRegister(motor, XTARGET, motor->registers.XTARGET.Val.Value);

	(void)TMC5160_wait_for_position(motor, target_position);

}

/*
 * @Brief Waits for XACTUAL to equal XTARGET.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param position: Desired position in usteps.
 * */
TMC5160_return_values_t TMC5160_wait_for_position(TMC5160_HandleTypeDef *motor, int position){
	// TODO: timeout may not give enough time to reach position. maybe not let the user set the timeout.

	uint32_t startTime = HAL_GetTick();

	SPI_Status_t  status = TMC5160_ReadRegister(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);

	if(status.Val.BitField.DRIVER_ERROR || status.Val.BitField.SG2){
		return status.Val.BitField.DRIVER_ERROR ? DRIVER_ERROR : STALL_ERROR;
	}
	uint32_t current_position = motor->registers.XACTUAL.Val.BitField.XACTUAL;

	int displacement; // TODO: subtract out XACTUAL and make it an absolute value
	uint32_t timeout;

	if(displacement < (16 * 1000)){
		timeout = 5000;
	}else{
		timeout = 1000 * (displacement / current_max_velocity);
	}



	while(current_position != position){
		(void)TMC5160_ReadRegister(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);
		current_position = motor->registers.XACTUAL.Val.BitField.XACTUAL;
		if((HAL_GetTick() - startTime) > timeout){
			return POSITION_NOT_REACHED;
		}
	}
	return TMC5160_SUCCESS;

}

/*
 * @Brief Resets the GSTAT register to clear any error flags.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
TMC5160_return_values_t TMC5160_reset_status(TMC5160_HandleTypeDef *motor){

	SPI_Status_t spi_res = TMC5160_ReadRegister(motor, &motor->registers.GSTAT.Val.Value, GSTAT);

	if(spi_res.Val.BitField.DRIVER_ERROR || spi_res.Val.BitField.SG2){
		return spi_res.Val.BitField.DRIVER_ERROR ? DRIVER_ERROR : STALL_ERROR;
	}

	// Reset flags must write 1 to clear
	motor->registers.GSTAT.Val.BitField.RESET = 1;
	motor->registers.GSTAT.Val.BitField.DRV_ERR = 1;
	motor->registers.GSTAT.Val.BitField.UV_CP = 1;
	if(TMC5160_WriteRegister(motor, GSTAT, motor->registers.GSTAT.Val.Value) != HAL_OK){
		return COMMUNICATION_ERROR;
	}

	spi_res = TMC5160_ReadRegister(motor, &motor->registers.GSTAT.Val.Value, GSTAT);

	if(motor->registers.GSTAT.Val.Value == 0){
		return TMC5160_SUCCESS;
	}else if(motor->registers.GSTAT.Val.BitField.DRV_ERR || motor->registers.GSTAT.Val.BitField.UV_CP ){
		return motor->registers.GSTAT.Val.BitField.DRV_ERR ? DRIVER_ERROR : UNDERVOLTAGE_ERROR;
	}else{
		return TMC5160_SUCCESS;
	}
}

///////////////////////////////////////////////////////////////////////////////////// PRIVATE FUNCTIONS ////////////////////////////////////

/*
 * @Brief Makes sure a requested register is readable using the register access lookup table.
 * @Param reg_addr: The address of the register to check.
 * */
static uint8_t is_readable(TMC5160_reg_addresses reg_addr) {

    for (size_t i = 0; i < reg_access_count; i++) {
        if (reg_access[i].addr == reg_addr) {
            return (uint8_t)(reg_access[i].access == READ ||
                    reg_access[i].access == READ_WRITE ||
                    reg_access[i].access == READ_WRITE_CLEAR);
        }
    }
    return 0x00; // Default to not readable if not found
}

/*
 * @Brief Calculates the GLOBAL_SCALAR Based on the sense resistor and desired current.
 * @Param desired_rms_current: The maximum RMS current of the stepper motor.
 * */
static uint8_t calculate_globalScaler(int  desired_rms_current){
	int GS = (int)((desired_rms_current * 256 * 32 * SENSE_RESISTOR *  SQRT_2) / ((CURRENT_SCALE + 1) * SENSE_VOLTAGE ));
	if(GS < 0) return 0;
	if(GS > 255) return 255;
	return (uint8_t)GS ;
}

/*
 * @Brief Performs a non-blocking delay.
 * @Param delay_ms: Desired delay in milliseconds.
 * */
static void rtos_delay(uint32_t delay_ms){
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

/*
 * @Brief Enables or disables the motor via the EN_PIN.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param state: desired pin state, 0 = motor on.
 * */
static void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state){
	HAL_GPIO_WritePin(motor->motor_en_port, motor->motor_en_pin, state);
}

/*
 * @Brief Converts the MRES register to base 10 micro-steps being used.
 * @Param MRES: The value of the micro-steps register.
 * */
uint16_t tmc_mres_to_microsteps(uint8_t mres)
{
    if (mres > 8) {
        mres = 8; // Clamp invalid MRES to 8 (1 microstep)
    }
    return 1 << (8 - mres); // 2^(8 - mres)
}


//TODO: Load all the registers using the lookup table




















