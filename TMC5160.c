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

/*
 * TODO:
 * 		Create a homing function
 * 		Zero Motor
 * 		Set Free Wheeling
 *		Currently monitoring the position blocks GUI functionality. May be time to start optimizing.
 *		Needs to communicate with multiple motors.

 *
 * */

#define PWM_FREQUENCY 		24000
#define MOTOR_VOLTAGE 		10
#define COIL_RESISTANCE 	1.6
#define FCLCK				12000000
#define MAX_VELOCITY		8000
#define MIN_VELOCITY		800

#define SENSE_RESISTOR  0.075
#define SENSE_VOLTAGE 	0.325
#define SQRT_2			1.41
#define CURRENT_SCALE	31
#define HOLD_CURRENT	16


////////////////////////////////////////////////////////////////////////////////////////////////////// PRIVATE VARIABLES ///////////////////////////////////////////////////////
static const size_t reg_access_count = sizeof(reg_access) / sizeof(reg_access[0]);
int ped_x_curr_buffer = 0;
TMC5160_error_indicator error_instance = {0};


 ///////////////////////////////////////////////////////////////////////////////////////////////////// PRIVATE FUNCTIONS /////////////////////////////////////////////////////
static void rtos_delay(uint32_t delay_ms);
static uint8_t is_readable(TMC5160_reg_addresses reg_addr);
static TMC5160_return_values_t TMC5160_reset_status(TMC5160_HandleTypeDef *motor);
static uint8_t calculate_globalScaler(int desired_rms_current);
static uint16_t tmc_mres_to_microsteps(uint8_t mres);
static HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor);
static TMC5160_return_values_t TMC5160_wait_for_position(TMC5160_HandleTypeDef *motor, int position);
static void TMC5160_set_velocity_mode(TMC5160_HandleTypeDef *motor,TMC5160_motor_directions direction );
static void TMC5160_velocity_mode_set_velocity(TMC5160_HandleTypeDef *motor, int velocity);
/////////////////////////////////////////////////////////////////////////////////////////////////////// PUBLIC FUNCTIONS //////////////////////////////////////////////////////

/*
 * @Brief Write initial motor settings.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
TMC5160_return_values_t TMC5160_init(TMC5160_HandleTypeDef *motor){

	// Disable motor for setup
	TMC5160_motor_enable(motor, GPIO_PIN_SET);

	// Reset flags on start
	if(TMC5160_reset_status(motor) != TMC5160_SUCCESS){
		error_instance.error = RESET_FAILURE;
		error_instance.error_flag = 1;
		return RESET_FAILURE;
	}

	// GLOBAL_SCALER
	motor->registers.GLOBAL_SCALER.Val.BitField.GLOBAL_SCALER = calculate_globalScaler(2);
	if(TMC5160_WriteRegister(motor, GLOBALSCALER, motor->registers.GLOBAL_SCALER.Val.Value)!= TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}

	// GCONF
	if(TMC5160_ReadRegisterUnsigned(motor, &motor->registers.GCONF.Val.Value, GCONF) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}
	motor->registers.GCONF.Val.BitField.EN_PWM_MODE = 1;
	if(TMC5160_WriteRegister(motor, GCONF, motor->registers.GCONF.Val.Value) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}

	// CHOPCONF
	if(TMC5160_ReadRegisterUnsigned(motor, &motor->registers.CHOPCONF.Val.Value, CHOPCONF)!= TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}
	motor->registers.CHOPCONF.Val.BitField.MRES  = ms_16;
	motor->registers.CHOPCONF.Val.BitField.TBL   = 1;
	motor->registers.CHOPCONF.Val.BitField.TOFF  = 4;
	motor->registers.CHOPCONF.Val.BitField.HEND  = 6;
	motor->registers.CHOPCONF.Val.BitField.HSTRT = 0;
	if(TMC5160_WriteRegister(motor, CHOPCONF, motor->registers.CHOPCONF.Val.Value) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}


	// PWMCONF
	motor->registers.PWMCONF.Val.BitField.PWM_AUTOSCALE = 1;
	motor->registers.PWMCONF.Val.BitField.PWM_AUTOGRAD = 1;
	motor->registers.PWMCONF.Val.BitField.PWM_FREQ = 0; // 24Khz
	motor->registers.PWMCONF.Val.BitField.FREEWHEEL = 0;
	if(TMC5160_WriteRegister(motor, PWMCONF, motor->registers.PWMCONF.Val.Value ) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}

	// IHOLD_IRUN
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLD = HOLD_CURRENT;
	motor->registers.IHOLD_IRUN.Val.BitField.IRUN = CURRENT_SCALE;
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLDDELAY = 3;
	if(TMC5160_WriteRegister(motor, IHOLD_IRUN, motor->registers.IHOLD_IRUN.Val.Value ) != TMC5160_SUCCESS ){
		return COMMUNICATION_ERROR;
	}

	// TPOWERDOWN
	motor->registers.TPOWERDOWN.Val.BitField.TPOWERDOWN = 3;
	if(TMC5160_WriteRegister(motor, TPOWERDOWN, motor->registers.TPOWERDOWN.Val.Value )!= TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}


	// Application is stealth chop only
	TMC5160_setup_stealthchop(motor);

	return TMC5160_SUCCESS;
}

/*
 * @Brief Check the status flags.
 * @Return uint8_t can be cast to SPI_Status_t
 * */
uint8_t TMC5160_get_status(TMC5160_HandleTypeDef *motor){
		uint8_t rx_data[5];
		uint8_t tx_data[5] = {GSTAT, 0x00, 0x00, 0x00, 0x00};

		// first response can be disposed of
		if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
			return COMMUNICATION_ERROR;
		}

		memset(rx_data, 0, sizeof(rx_data) );

		if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
			return COMMUNICATION_ERROR;
		}else{
			return rx_data[0];
		}
}

/*
 * @Brief Changes the motor's travel speed.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param velocity: new maximum travel velocity in usteps/s
 * */
void TMC5160_set_max_velocity(TMC5160_HandleTypeDef *motor, int velocity){

	if(velocity > MAX_VELOCITY) {
		velocity = MAX_VELOCITY;
	}else if (velocity > 0 && velocity < MIN_VELOCITY){
		velocity = MIN_VELOCITY;
	}else if(velocity < 0){
		velocity = 0;
	}

	// Speeds for positioning mode
	motor->registers.A1.Val.BitField.A1 =  (uint16_t)(0.5 * velocity);
	(void)TMC5160_WriteRegister(motor, A1, motor->registers.A1.Val.Value);

	motor->registers.V1.Val.BitField.V1 = (uint32_t)(0.15 * velocity);
	(void)TMC5160_WriteRegister(motor, V1, motor->registers.V1.Val.Value);

	motor->registers.AMAX.Val.BitField.AMAX = (uint16_t)(0.6 * velocity);
	(void)TMC5160_WriteRegister(motor, AMAX, motor->registers.AMAX.Val.Value);

	motor->registers.DMAX.Val.BitField.DMAX = motor->registers.AMAX.Val.BitField.AMAX;
	(void)TMC5160_WriteRegister(motor, DMAX, motor->registers.DMAX.Val.Value);

	motor->registers.D1.Val.BitField.D1 = motor->registers.A1.Val.BitField.A1;
	(void)TMC5160_WriteRegister(motor, D1, motor->registers.D1.Val.Value);

	motor->registers.VMAX.Val.BitField.VMAX = velocity;
	(void)TMC5160_WriteRegister(motor, VMAX, motor->registers.VMAX.Val.Value);
}

/*
 @Brief Moves the motor the number of full steps requested. Forward or reverse is determined by the sign
 	 	of the requested number.
 @Param motor : Pointer to the motor struct.
 @Param full_steps : Positive or negative number of steps to move.
 * */
void TMC5160_move_to_position(TMC5160_HandleTypeDef *motor, int full_steps){

	// Get the microsteps for total steps to write
	uint16_t microsteps =  tmc_mres_to_microsteps((uint8_t)motor->registers.CHOPCONF.Val.BitField.MRES);

	(void)TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);
	int current_position = motor->registers.XACTUAL.Val.BitField.XACTUAL;
	int total_steps = microsteps * full_steps;
	int target_position;

	TMC5160_set_positioning_mode(motor);

	if((current_position + total_steps) <= 0 ){
		target_position = 0;
	}else{
		target_position = current_position + total_steps;
	}

	// setting target position to the current position does not trigger the position reached flag.
	if(target_position == 0 && current_position == 0){
		return;
	}

	motor->registers.XTARGET.Val.BitField.XTARGET = target_position;
	(void)TMC5160_WriteRegister(motor, XTARGET, motor->registers.XTARGET.Val.Value);

	(void)TMC5160_wait_for_position(motor, target_position);
	(void)TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);
	ped_x_curr_buffer = motor->registers.XACTUAL.Val.Value / microsteps;
}

/*
 * @Brief Resets the GSTAT register to clear any error flags.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
TMC5160_return_values_t TMC5160_reset_status(TMC5160_HandleTypeDef *motor){

	if(TMC5160_ReadRegisterUnsigned(motor, &motor->registers.GSTAT.Val.Value, GSTAT) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}

	// Reset flags must write 1 to clear
	motor->registers.GSTAT.Val.BitField.RESET = 1;
	motor->registers.GSTAT.Val.BitField.DRV_ERR = 1;
	motor->registers.GSTAT.Val.BitField.UV_CP = 1;
	if(TMC5160_WriteRegister(motor, GSTAT, motor->registers.GSTAT.Val.Value) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}

	if(TMC5160_ReadRegisterUnsigned(motor, &motor->registers.GSTAT.Val.Value, GSTAT) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}

	if(motor->registers.GSTAT.Val.Value == 0){
		return TMC5160_SUCCESS;
	}else if(motor->registers.GSTAT.Val.BitField.DRV_ERR || motor->registers.GSTAT.Val.BitField.UV_CP ){
		return motor->registers.GSTAT.Val.BitField.DRV_ERR ? DRIVER_ERROR : UNDERVOLTAGE_ERROR;
	}else{
		return TMC5160_SUCCESS;
	}
}

void TMC5160_set_positioning_mode(TMC5160_HandleTypeDef *motor){
	if(motor->registers.RAMPMODE.Val.BitField.RAMPMODE != 0){
		TMC5160_motor_enable(motor, GPIO_PIN_SET);
		motor->registers.RAMPMODE.Val.BitField.RAMPMODE = 0;
		(void)TMC5160_WriteRegister(motor, RAMPMODE, motor->registers.RAMPMODE.Val.Value);
		TMC5160_motor_enable(motor, GPIO_PIN_RESET);
	}

}

/*
 * @Brief Enables or disables the motor via the EN_PIN.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param state: desired pin state, 0 = motor on.
 * */
void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state){
	HAL_GPIO_WritePin(motor->motor_en_port, motor->motor_en_pin, state);
}

/*
 @Brief Writes data to the desired register.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to write to.
 @Param *data: Pointer to the data to be written, should be a 4 byte array.
 @Ret HAL_StatusTypeDef
 * */
TMC5160_return_values_t TMC5160_WriteRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data){
	//TODO: Needs motor ID and to write to different motor controllers

	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		return NULL_MOTOR_ERROR;
	}
	while(motor->spi->State != HAL_SPI_STATE_READY){rtos_delay(1);}

	uint8_t tx_data[5];
	tx_data[0] = (reg_addr | WRITE_MASK);
	tx_data[1] = (uint8_t)(data >> 24 );
	tx_data[2] = (uint8_t)(data >> 16 );
	tx_data[3] = (uint8_t)(data >> 8 );
	tx_data[4] = (uint8_t)(data);

	 if(HAL_SPI_Transmit(motor->spi, tx_data, sizeof(tx_data), HAL_MAX_DELAY) != HAL_OK){
		 return COMMUNICATION_ERROR;
	 }else{
		 rtos_delay(1);
		 return TMC5160_SUCCESS;
	 }



}

/*
 @Brief reads the requested register and returns the value.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to read.
 @Ret uint32_t: Either the register value or 0xFFFFFFFF indicating an error.
 * */
TMC5160_return_values_t TMC5160_ReadRegisterUnsigned(TMC5160_HandleTypeDef *motor, uint32_t * reg_val, uint8_t reg_addr){
	//TODO: Needs motor ID and to read from the different motor controllers
	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		return NULL_MOTOR_ERROR;
	}

	if(!is_readable(reg_addr)){
		return REGISTER_ACCESS_ERROR;
	}


	uint8_t rx_data[5];
	uint8_t tx_data[5] = {reg_addr, 0x00, 0x00, 0x00, 0x00};

	// first response can be disposed of
	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}

	memset(rx_data, 0, sizeof(rx_data) );

	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}else{
		*reg_val = ( (rx_data[1] << 24) | (rx_data[2] << 16)| (rx_data[3] << 8) | rx_data[4] );
		return TMC5160_SUCCESS;
	}

}

/*
 @Brief reads the requested register and returns the value.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to read.
 @Ret uint32_t: Either the register value or 0xFFFFFFFF indicating an error.
 * */
TMC5160_return_values_t TMC5160_ReadRegisterSigned(TMC5160_HandleTypeDef *motor, int32_t * reg_val, uint8_t reg_addr){
	//TODO: Needs motor ID and to read from the different motor controllers
	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		return NULL_MOTOR_ERROR;
	}

	if(!is_readable(reg_addr)){
		return REGISTER_ACCESS_ERROR;
	}


	uint8_t rx_data[5];
	uint8_t tx_data[5] = {reg_addr, 0x00, 0x00, 0x00, 0x00};

	// first response can be disposed of
	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;;
	}

	memset(rx_data, 0, sizeof(rx_data) );

	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}else{
		*reg_val = ( (rx_data[1] << 24) | (rx_data[2] << 16)| (rx_data[3] << 8) | rx_data[4] );
		return TMC5160_SUCCESS;
	}

}

// TODO: This is temporary
TMC5160_return_values_t TMC5160_set_zero(TMC5160_HandleTypeDef *motor){

	// Get the current position
	TMC5160_return_values_t res = TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);
	if(res == TMC5160_SUCCESS)
	{
	// Disable the motor to switch modes
	TMC5160_motor_enable(motor, GPIO_PIN_SET);
	TMC5160_velocity_mode_set_velocity(motor, 0);
	TMC5160_set_velocity_mode(motor, FORWARD);

	res = TMC5160_ReadRegisterUnsigned(motor, &motor->registers.RAMPMODE.Val.Value, RAMPMODE);
		if( res == TMC5160_SUCCESS)
		{
			if(motor->registers.XACTUAL.Val.Value == 0)
			{
				// first move forward a bit and return hitting the limit switch to confirm.
				TMC5160_velocity_mode_set_velocity(motor, MAX_VELOCITY);
				rtos_delay(2000);

				TMC5160_velocity_mode_set_velocity(motor, 0);
				TMC5160_set_velocity_mode(motor, REVERSE);
				rtos_delay(2000);

				TMC5160_velocity_mode_set_velocity(motor, MAX_VELOCITY);
				rtos_delay(2000);
				TMC5160_velocity_mode_set_velocity(motor, 0);
				TMC5160_set_velocity_mode(motor, FORWARD);


				// check if position is still 0 if so return else adjust.
				res = TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);
				if(motor->registers.XACTUAL.Val.Value == 0){
					TMC5160_set_positioning_mode(motor);
					return TMC5160_SUCCESS;
				}else{
					TMC5160_motor_enable(motor, GPIO_PIN_SET);
					TMC5160_WriteRegister(motor, XACTUAL, 0);
					TMC5160_motor_enable(motor, GPIO_PIN_RESET);
					TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);
					int xactual_val = motor->registers.XACTUAL.Val.Value;

					TMC5160_set_positioning_mode(motor);

					TMC5160_set_max_velocity(motor, MAX_VELOCITY);
					if(xactual_val == 0 ){
						return TMC5160_SUCCESS;
					}else{
						return COMMUNICATION_ERROR;
					}
				}
			}else
			{
				// Move back the amount the XACTUL says it is.
				// wait for switch to confirm if switch/ XACTUAL out of sync adjust.
				return res;
			}
		}else
		{
			return res;
		}
	}else
	{
	return res;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////// PRIVATE FUNCTIONS ////////////////////////////////////////////////////////

/*
 * @Brief Tune StealthChop for quiet motor movement.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
static HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor){
	/*
	 * TODO: Homing using stall guard
	 * */

	motor->registers.TPWMTHRS.Val.BitField.TPWMTHRS = 0;

	TMC5160_WriteRegister(motor, TPWMTHRS, motor->registers.TPWMTHRS.Val.Value);

	TMC5160_set_max_velocity(motor, MAX_VELOCITY); //8000

	TMC5160_motor_enable(motor, GPIO_PIN_RESET);

	TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL);

	if(motor->registers.XACTUAL.Val.Value > 0){
		TMC5160_move_to_position(motor, -motor->registers.XACTUAL.Val.Value);
	}else{
		rtos_delay(10);
		TMC5160_move_to_position(motor, 200);
		rtos_delay(100);
		TMC5160_move_to_position(motor, -200);
	}

	return HAL_OK;
}

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
	/* Current Setting:
		IRMS = (GLOBALSCALER \ 256) * (CS+1 \ 32) * (VFS / RSENSE) * (1/ sqrt(2))
		GLOBALSCALER = (IRMS * 256 * 32 * RSENSE * sqrt(2)) /  (CS+1) * VFS
	 * */
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
 * @Brief Waits for XACTUAL to equal XTARGET.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param position: Desired position in usteps.
 * */
static TMC5160_return_values_t TMC5160_wait_for_position(TMC5160_HandleTypeDef *motor, int position){

	// Start timer for timeout operations
	uint32_t startTime = HAL_GetTick();

	// max time in milliseconds to move the full distance at minimum speed.
	uint32_t timeout  = 500000;

	while(!(motor->registers.XACTUAL.Val.Value == motor->registers.XTARGET.Val.Value)){
		if(TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL) != TMC5160_SUCCESS){
			return COMMUNICATION_ERROR;
		}

		ped_x_curr_buffer = (motor->registers.XACTUAL.Val.Value /  (uint32_t)tmc_mres_to_microsteps((uint8_t)motor->registers.CHOPCONF.Val.BitField.MRES) );
		if((HAL_GetTick() - startTime) > timeout){
			return POSITION_NOT_REACHED;
		}
	}
	if(TMC5160_ReadRegisterSigned(motor, &motor->registers.XACTUAL.Val.Value, XACTUAL) != TMC5160_SUCCESS){
		return COMMUNICATION_ERROR;
	}
	return TMC5160_SUCCESS;

}

/*
 * @Brief Converts the MRES register to base 10 micro-steps being used.
 * @Param MRES: The value of the micro-steps register.
 * */
static uint16_t tmc_mres_to_microsteps(uint8_t mres)
{
    return 1 << mres;
}


static void TMC5160_set_velocity_mode(TMC5160_HandleTypeDef *motor,TMC5160_motor_directions direction ){

		if(motor->registers.VSTOP.Val.BitField.VSTOP < 1){
			motor->registers.VSTOP.Val.BitField.VSTOP = 100;
			motor->registers.VSTART.Val.BitField.VSTART = 0;
			TMC5160_WriteRegister(motor, VSTART, 0);
			TMC5160_WriteRegister(motor, VSTOP, 100);
		}

		TMC5160_motor_enable(motor, GPIO_PIN_SET);
		motor->registers.RAMPMODE.Val.BitField.RAMPMODE = direction;
		(void)TMC5160_WriteRegister(motor, RAMPMODE, motor->registers.RAMPMODE.Val.Value);
		TMC5160_motor_enable(motor, GPIO_PIN_RESET);

}


static void TMC5160_velocity_mode_set_velocity(TMC5160_HandleTypeDef *motor, int velocity){
	motor->registers.VMAX.Val.BitField.VMAX = velocity;
	(void)TMC5160_WriteRegister(motor, VMAX, motor->registers.VMAX.Val.Value);
}



















