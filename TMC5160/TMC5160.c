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
#define BLANK_TIME		24
#define SENSE_RESISTOR  .075



static const size_t reg_access_count = sizeof(reg_access) / sizeof(reg_access[0]);


typedef enum{
	TMC5160_SUCCESS 		= 0x00,
	NULL_MOTOR_ERROR 		= 0x01,
	REGISTER_ACCESS_ERROR 	= 0x02,
	COMMUNICATION_ERROR 	= 0x03,
	DRIVER_ERROR			= 0x04,
	STALL_ERROR				= 0x05,
	UNDERVOLTAGE_ERROR		= 0x06
}TMC5160_return_values_t;


static void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state);
static void rtos_delay(uint32_t delay_ms);
static uint8_t is_readable(TMC5160_reg_addresses reg_addr);
static TMC5160_return_values_t TMC5160_reset_status(TMC5160_HandleTypeDef *motor);



/* Current Setting:
 * Set GLOBALSCALER as required to reach maximum motor current at I_RUN=31   NOTE CSActual needed for calculation is in coolstep
 * Set I_RUN as desired up to 31, I_HOLD 70% of I_RUN or lower
 * Set I_HOLD_DELAY to 1 to 15 for smooth standstill current decay
 * Set TPOWERDOWN up to 255 for delayed standstill current reduction
 * Configure Chopper to test current settings
 *
 *
 *  StealthChop Configuration:
 *	GCONF set en_pwm_mode
 *	PWMCONF set pwm_autoscale, set pwm_autograd
 *	PWMCONF select PWM_FREQ with regard to fCLK for 20- 40kHz PWM frequency
 *	CHOPCONF Enable chopper using basic config., e.g.: TOFF=5, TBL=2, HSTART=4, HEND=0
 *	Execute automatic tuning procedure AT
 *	Move the motor by slowly accelerating from 0 to VMAX operation velocity
 * */







HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor){



	// Disable motor for setup
	TMC5160_motor_enable(motor, GPIO_PIN_SET);

	SPI_Status_t spi_res = TMC5160_ReadRegister(motor, &motor->registers.GSTAT.Val.Value, GSTAT);

	// Reset flags on start
	(void)TMC5160_reset_status(motor);

	// GLOBAL_SCALER
	motor->registers.GLOBAL_SCALER.Val.BitField.GLOBAL_SCALER = 167;
	(void)TMC5160_WriteRegister(motor, GLOBAL_SCALER, motor->registers.GLOBAL_SCALER.Val.Value);

	// GCONF
	spi_res = TMC5160_ReadRegister(motor, &motor->registers.GCONF.Val.Value, GCONF);
	motor->registers.GCONF.Val.BitField.EN_PWM_MODE = 1;
	(void)TMC5160_WriteRegister(motor, GCONF, motor->registers.GCONF.Val.Value);

	// CHOPCONF
	spi_res = TMC5160_ReadRegister(motor, &motor->registers.CHOPCONF.Val.Value, CHOPCONF);
	motor->registers.CHOPCONF.Val.BitField.MRES  = 4; //16
	motor->registers.CHOPCONF.Val.BitField.TBL   = 1;
	motor->registers.CHOPCONF.Val.BitField.TOFF  = 4;
	motor->registers.CHOPCONF.Val.BitField.HEND  = 0;
	motor->registers.CHOPCONF.Val.BitField.HSTRT = 4;
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
	motor->registers.IHOLD_IRUN.Val.BitField.IRUN = 29;
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLDDELAY = 4;
	(void)TMC5160_WriteRegister(motor, IHOLD_IRUN, motor->registers.IHOLD_IRUN.Val.Value );

	// TPOWERDOWN
	motor->registers.TPOWERDOWN.Val.BitField.TPOWERDOWN = 3;
	(void)TMC5160_WriteRegister(motor, TPOWERDOWN, motor->registers.TPOWERDOWN.Val.Value );

	// RAMP MODE

/*     µstep velocity µsteps / s:
 * 			 VMAX_LIMIT = 8388096
			 v[Hz] = v[TMC] * ( fCLK[Hz]/ 2^24 )
			 v[TMC] = ( v[Hz] * 2^24 ) / fCLK
 	 	 	 1,118,481 = 1600 * 2^24 / 12MHz      if its wrong try it with fCLK = 24000
 	 	 	 1600 Hz = 2236

		µstep acceleration a[Hz/s]:
			a[Hz/s] = a[TMC] * fCLK[Hz]^2 / ((512*256) * 2^24)
			a[TMC] = a[Hz/s]  * (( 512 * 256) * 2^24) / fCLK[Hz]^2
			2,443,359 = 640 * (( 512 * 256) * 2^24 / 12MHz^2     if its wrong try it with fCLK = 24000
			10 = 640 Hz

*/
	motor->registers.VMAX.Val.BitField.VMAX = 2236; // 1600 Hz
	(void)TMC5160_WriteRegister(motor, VMAX, motor->registers.VMAX.Val.Value);

	motor->registers.AMAX.Val.BitField.AMAX = 10;
	(void)TMC5160_WriteRegister(motor, AMAX, motor->registers.AMAX.Val.Value);


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

static void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state){
	HAL_GPIO_WritePin(motor->motor_en_port, motor->motor_en_pin, state);
}


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


// Check if a register is readable
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

static void rtos_delay(uint32_t delay_ms){
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
}




//TODO: Load all the registers using the lookup table




















