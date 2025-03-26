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
 *
 *
 * FCLK = 12MHz
Time reference t for velocities: t = 2^24 / fCLK
Time reference ta² for accelerations: ta² = 2^41 / (fCLK)² = 0.009

SPI send: 0xA4000003E8; // A1 = 1 000 First acceleration
SPI send: 0xA50000C350; // V1 = 50 000 Acceleration threshold velocity V1
SPI send: 0xA6000001F4; // AMAX = 500 Acceleration above V1
SPI send: 0xA700030D40; // VMAX = 200 000
SPI send: 0xA8000002BC; // DMAX = 700 Deceleration above V1
SPI send: 0xAA00000578; // D1 = 1400 Deceleration below V1
SPI send: 0xAB0000000A; // VSTOP = 10 Stop velocity (Near to zero)
SPI send: 0xA000000000; // RAMPMODE = 0 (Target position move)
// Ready to move!
SPI send: 0xADFFFF3800; // XTARGET = -51200 (Move one rotation left (200*256 microsteps)
// Now motor 1 starts rotating
SPI send: 0x2100000000; // Query XACTUAL – The next read access delivers XACTUAL
SPI read; // Read XACTUAL


Set the motion profile
(e.g., AMAX, DMAX, VMAX registers for acceleration, deceleration, and velocity).

Configure the motor current
(e.g., IRUN and IHOLD in the IHOLD_IRUN register).

Choose a microstep resolution
(via the CHOPCONF register).

Optionally enable features like StealthChop2 for quiet operation or
CoolStep for energy efficiency.


enable en_pwm_mode bit),

send postion and velocity.

*/



void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state);
void rtos_delay(uint32_t delay_ms);
uint32_t read_status(TMC5160_HandleTypeDef *motor);

HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor){


	// Current settings
	// fPWM = 24Khz
	// VM =  10v
	// RCOIL = 1.6 Ohms
	// tBlank = 24 cycles
	// ILower_Limit = tBlank* fPWM * VM / RCOIL
	// 24 * 2/1024 * 10/1.6 = .293A

	// Disable motor for setup
	TMC5160_motor_enable(motor, GPIO_PIN_RESET);
	motor->registers.GSTAT.Val.Value = read_status(motor);
	motor->registers.GSTAT.Val.BitField.RESET = 1;
	motor->registers.GSTAT.Val.BitField.DRV_ERR = 1;
	motor->registers.GSTAT.Val.BitField.UV_CP = 1;
	(void)TMC5160_WriteRegister(motor, GSTAT, motor->registers.GSTAT.Val.Value);

	motor->registers.GSTAT.Val.Value = read_status(motor);

	// GCONF
	uint32_t gconf_read_val = TMC5160_ReadRegister(motor, GCONF);
	if(gconf_read_val == 0xFFFFFFFF){
		return HAL_ERROR;
	}
	motor->registers.GCONF.Val.Value = gconf_read_val;
	motor->registers.GCONF.Val.BitField.EN_PWM_MODE = 1;
	(void)TMC5160_WriteRegister(motor, GCONF, motor->registers.GCONF.Val.Value);
	motor->registers.GSTAT.Val.Value = read_status(motor);

	// PWMCONF
	uint32_t pwmconf_read_val = TMC5160_ReadRegister(motor, PWMCONF);
	if(pwmconf_read_val == 0xFFFFFFFF){
		return HAL_ERROR;
	}
	motor->registers.PWMCONF.Val.Value = pwmconf_read_val;
	motor->registers.PWMCONF.Val.BitField.PWM_AUTOSCALE = 1;
	motor->registers.PWMCONF.Val.BitField.PWM_FREQ = 2;
	motor->registers.PWMCONF.Val.BitField.PWM_AUTOGRAD = 1;
	(void)TMC5160_WriteRegister(motor, PWMCONF, motor->registers.PWMCONF.Val.Value );
	uint32_t pwm_res = TMC5160_ReadRegister(motor, PWMCONF);
	motor->registers.GSTAT.Val.Value = read_status(motor);

	// IHOLD_IRUN
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLD = 31;
	motor->registers.IHOLD_IRUN.Val.BitField.IRUN = 31;
	motor->registers.IHOLD_IRUN.Val.BitField.IHOLDDELAY = 0;
	(void)TMC5160_WriteRegister(motor, IHOLD_IRUN, motor->registers.IHOLD_IRUN.Val.Value );
	uint32_t ihold_res = TMC5160_ReadRegister(motor, IHOLD_IRUN);
	motor->registers.GSTAT.Val.Value = read_status(motor);

	// CHOPCONF
	motor->registers.CHOPCONF.Val.BitField.TBL = 2;
	(void)TMC5160_WriteRegister(motor, CHOPCONF, motor->registers.CHOPCONF.Val.Value);
	uint32_t chop_res = TMC5160_ReadRegister(motor, CHOPCONF);
	motor->registers.GSTAT.Val.Value = read_status(motor);

	TMC5160_motor_enable(motor, GPIO_PIN_RESET);
	rtos_delay(200);

	// move


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

	return HAL_SPI_Transmit(motor->spi, tx_data, sizeof(tx_data), HAL_MAX_DELAY);

}

/*
 @Brief reads the requested register and returns the value.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to read.
 @Ret uint32_t: Either the register value or 0xFFFFFFFF indicating an error.
 * */
uint32_t TMC5160_ReadRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr){

	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		return 0xFFFFFFFF;
	}


	uint8_t rx_data[5];
	uint8_t tx_data[5] = {reg_addr, 0x00, 0x00, 0x00, 0x00};

	// first response can be disposed of
	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return 0xFFFFFFFF;
	}

	memset(rx_data, 0, sizeof(rx_data) );

	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return 0xFFFFFFFF;
	}else{
		uint32_t recieved = ( (rx_data[1] << 24) | (rx_data[2] << 16)| (rx_data[3] << 8) | rx_data[4] );
		return recieved;
	}
}

void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state){
	HAL_GPIO_WritePin(motor->motor_en_port, motor->motor_en_pin, state);
}

void rtos_delay(uint32_t delay_ms){
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

uint32_t read_status(TMC5160_HandleTypeDef *motor){
	uint32_t gstat_read_val = TMC5160_ReadRegister(motor, GSTAT);
	if(gstat_read_val == 0xFFFFFFFF){
		return 0xFFFFFFFF;
	}
	return gstat_read_val;
}

/*
  Quick Setup:

  Current setting

  Set GLOBALSCALER as required to reach maximum motor current at I_RUN=31:

  Set I_RUN as desired up to 31, I_HOLD 70% of I_RUN or lower

  Set I_HOLD_DELAY to 1 to 15 for smooth standstill current decay

  Set TPOWERDOWN up to 255 for delayed standstill current reduction

  Configure Chopper to test current settings

  stealthChop Configuration

  GCONF set en_pwm_mode

  PWMCONF set pwm_autoscale, set pwm_autograd

  PWMCONF select PWM_FREQ with regard to fCLK for 20- 40kHz PWM frequency

  CHOPCONF Enable chopper using basic config.,
  e.g.: TOFF=5, TBL=2, HSTART=4, HEND=0

  Execute automatic tuning procedure AT

  Move the motor by slowly accelerating from 0 to VMAX operation velocity

  Is performance good up to VMAX?
  	  	  Y: done
  	  	  N: Select a velocity threshold for switching to spreadCycle chopper and set TPWMTHRS

 * */










