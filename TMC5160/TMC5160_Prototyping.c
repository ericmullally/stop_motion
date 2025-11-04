/*
 * TMC5160_Prototyping.c
 *
 *  Created on: Jul 21, 2025
 *      Author: Eric
 */

#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include "TMC5160_Structs.h"
#include "TMC5160.h"



#define WRITE_MASK			0x80
#define NUM_MOTORS			4
#define DATA_PACKET_SIZE 	5

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

extern TMC5160_HandleTypeDef motors[NUM_MOTORS];

static const size_t reg_offsets[128] = {
		[GCONF] = offsetof(TMC_Registers, GCONF.Val.Value),
		[GSTAT] = offsetof(TMC_Registers, GSTAT.Val.Value),
		[IOIN] = offsetof(TMC_Registers, IOIN.Val.Value),
		[OTP_READ] = offsetof(TMC_Registers, OTP_READ.Val.Value),
		[FACTORY_CONF] = offsetof(TMC_Registers, FACTORY_CONF.Val.Value),
		[OFFSET_READ] = offsetof(TMC_Registers, OFFSET_READ.Val.Value),
		[TSTEP]  = offsetof(TMC_Registers, TSTEP.Val.Value),
		[RAMPMODE] = offsetof(TMC_Registers, RAMPMODE.Val.Value),
		[XACTUAL]= offsetof(TMC_Registers, XACTUAL.Val.Value),
		[VACTUAL] = offsetof(TMC_Registers, VACTUAL.Val.Value),
		[XTARGET] = offsetof(TMC_Registers, XTARGET.Val.Value),
		[SW_MODE] = offsetof(TMC_Registers, SW_MODE.Val.Value),
		[RAMP_STAT] = offsetof(TMC_Registers, RAMP_STAT.Val.Value),
		[ENCMODE] =  offsetof(TMC_Registers, ENCMODE.Val.Value),
		[X_ENC] =  offsetof(TMC_Registers, X_ENC.Val.Value),
		[ENC_STATUS] = offsetof(TMC_Registers, ENC_STATUS.Val.Value),
		[ENC_LATCH] = offsetof(TMC_Registers, ENC_LATCH.Val.Value),
		[CHOPCONF] = offsetof(TMC_Registers, CHOPCONF.Val.Value),
		[DRV_STATUS] = offsetof(TMC_Registers, DRV_STATUS.Val.Value),
		[PWM_SCALE] = offsetof(TMC_Registers, PWM_SCALE.Val.Value),
		[PWM_AUTO] = offsetof(TMC_Registers, PWM_AUTO.Val.Value)
};


TMC5160_return_values_t TMC5160_init_p(TMC5160_HandleTypeDef *motor);
TMC5160_return_values_t TMC5160_WriteRegister_p(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data, uint8_t all_motors);
TMC5160_return_values_t TMC5160_ReadRegister_p(TMC5160_HandleTypeDef *motor, uint8_t reg_addr);
TMC5160_return_values_t TMC5160_ReadRegister_all_motors(SPI_HandleTypeDef *spi, uint8_t reg_addr);
TMC5160_return_values_t TMC5160_set_zero_p(TMC5160_HandleTypeDef *motor);
void TMC5160_set_max_velocity_p(TMC5160_HandleTypeDef *motor, int velocity);
void TMC5160_move_to_position_p(TMC5160_HandleTypeDef *motor, int full_steps);
void TMC5160_set_positioning_mode_p(TMC5160_HandleTypeDef *motor);
void TMC5160_motor_enable_p(TMC5160_HandleTypeDef *motor, GPIO_PinState state);
uint8_t TMC5160_get_status_p(TMC5160_HandleTypeDef *motor);

static void rtos_delay_p(uint32_t delay_ms);
static uint8_t is_readable_p(TMC5160_reg_addresses reg_addr);
static TMC5160_return_values_t TMC5160_reset_status_p(TMC5160_HandleTypeDef *motor);
static uint8_t calculate_globalScaler_p(int desired_rms_current);
static uint16_t tmc_mres_to_microsteps_p(uint8_t mres);
static HAL_StatusTypeDef TMC5160_setup_stealthchop_p(TMC5160_HandleTypeDef *motor);
static TMC5160_return_values_t TMC5160_wait_for_position_p(TMC5160_HandleTypeDef *motor, int position);
static void TMC5160_set_velocity_mode_p(TMC5160_HandleTypeDef *motor,TMC5160_motor_directions direction );
static void TMC5160_velocity_mode_set_velocity_p(TMC5160_HandleTypeDef *motor, int velocity);
static uint32_t* get_reg_ptr(uint8_t motor_index, uint8_t reg_addr);

static const size_t reg_access_count = sizeof(reg_access) / sizeof(reg_access[0]);
extern int ped_x_curr_buffer;
extern TMC5160_error_indicator error_instance;


/*
 * @Brief Write initial motor settings.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
TMC5160_return_values_t TMC5160_init_p(TMC5160_HandleTypeDef *motor_arr ){

	// Disable motor for setup
	TMC5160_motor_enable_p(&motor_arr[0], GPIO_PIN_SET);

	for (int i = 0; i < sizeof(motor_arr); i++){
		motor_arr[i].ID = i;
	}

	TMC5160_ReadRegister_p(&motor_arr[0], GSTAT);
	TMC5160_ReadRegister_p(&motor_arr[0], GSTAT);
	return TMC5160_SUCCESS;
}

TMC5160_return_values_t TMC5160_reset_status_p(TMC5160_HandleTypeDef *motor){

}

/*
 @Brief Writes data to the desired register.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to write to.
 @Param *data: Pointer to the data to be written, should be a 4 byte array.
 @Ret HAL_StatusTypeDef
 * */
TMC5160_return_values_t TMC5160_WriteRegister_p(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data, uint8_t all_motors){

	if(motor == NULL || motor->spi == NULL || motor->spi->Instance == NULL){
		return NULL_MOTOR_ERROR;
	}
	uint8_t full_packet[DATA_PACKET_SIZE * NUM_MOTORS] = {0}; // [1...20] motors 1->4

	if(all_motors){
		int motor_index =0;
		while(motor_index < (NUM_MOTORS - 1 * DATA_PACKET_SIZE) ){
			full_packet[motor_index] = (reg_addr | WRITE_MASK);
			full_packet[motor_index + 1] = (uint8_t)(data >> 24 );
			full_packet[motor_index + 2] = (uint8_t)(data >> 16 );
			full_packet[motor_index + 3] = (uint8_t)(data >> 8 );
			full_packet[motor_index + 4] = (uint8_t)(data);

			motor_index += DATA_PACKET_SIZE;
		}

	}else{
			// Identify which motor we are talking to.
		switch(motor->ID){
		case 0:
			full_packet[15] = (reg_addr | WRITE_MASK);
			full_packet[16] = (uint8_t)(data >> 24 );
			full_packet[17] = (uint8_t)(data >> 16 );
			full_packet[18] = (uint8_t)(data >> 8  );
			full_packet[19] = (uint8_t)(data);
			break;
		case 1:
			full_packet[10] = (reg_addr | WRITE_MASK);
			full_packet[11] = (uint8_t)(data >> 24 );
			full_packet[12] = (uint8_t)(data >> 16 );
			full_packet[13] = (uint8_t)(data >> 8  );
			full_packet[14] = (uint8_t)(data);
			break;
		case 2:
			full_packet[5] = (reg_addr | WRITE_MASK);
			full_packet[6] = (uint8_t)(data >> 24 );
			full_packet[7] = (uint8_t)(data >> 16 );
			full_packet[8] = (uint8_t)(data >> 8  );
			full_packet[9] = (uint8_t)(data);
			break;
		case 3:
			full_packet[0] = (reg_addr | WRITE_MASK);
			full_packet[1] = (uint8_t)(data >> 24 );
			full_packet[2] = (uint8_t)(data >> 16 );
			full_packet[3] = (uint8_t)(data >> 8 );
			full_packet[4] = (uint8_t)(data);
			break;
		default:
			return NULL_MOTOR_ERROR;
		}
	}

	while(motor->spi->State != HAL_SPI_STATE_READY){rtos_delay_p(1);}


	 if(HAL_SPI_Transmit(motor->spi, full_packet, sizeof(full_packet), HAL_MAX_DELAY) != HAL_OK){
		 return COMMUNICATION_ERROR;
	 }else{
		 rtos_delay_p(1);
		 return TMC5160_SUCCESS;
	 }

}

/*
 @Brief reads the requested register and returns the value.

 @Param *motor: Pointer to TMC5160_HandleTypeDef.
 @Param reg_addr: Address of the register you wish to read.
 @Ret uint32_t: Either the register value or 0xFFFFFFFF indicating an error.
 * */
TMC5160_return_values_t TMC5160_ReadRegister_p(TMC5160_HandleTypeDef *motor, uint8_t reg_addr){

	if(!is_readable_p(reg_addr)){
		return REGISTER_ACCESS_ERROR;
	}

	uint8_t rx_data[DATA_PACKET_SIZE * NUM_MOTORS] = {0};
	uint8_t tx_data[DATA_PACKET_SIZE * NUM_MOTORS] = {0};

	int address_index = 0;
	while(address_index <= ((NUM_MOTORS - 1) * DATA_PACKET_SIZE) ){
		tx_data[address_index] = reg_addr;
		address_index += DATA_PACKET_SIZE;
	}

	// first response can be disposed of
	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}

	if(HAL_SPI_TransmitReceive(motor->spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}else{
		uint32_t* reg_pointer = get_reg_ptr(motor->ID, reg_addr);

		switch(motor->ID){
		case 0:
			*reg_pointer = ( (rx_data[16] << 24) |(rx_data[17] << 16)| (rx_data[18] << 8) | rx_data[19] );
			break;
		case 1:
			*reg_pointer = ( (rx_data[11] << 24) |(rx_data[12] << 16)| (rx_data[13] << 8) | rx_data[14] );
			break;
		case 2:
			*reg_pointer = ( (rx_data[6] << 24) | (rx_data[7] << 16)| (rx_data[8] << 8) | rx_data[9] );
			break;
		case 3:
			*reg_pointer = ( (rx_data[1] << 24) | (rx_data[2] << 16)| (rx_data[3] << 8) | rx_data[4] );
			break;
		default:
			return NULL_MOTOR_ERROR;
		}

		return TMC5160_SUCCESS;
	}

}

TMC5160_return_values_t TMC5160_ReadRegister_all_motors(SPI_HandleTypeDef *spi, uint8_t reg_addr){

	if(!is_readable_p(reg_addr)){
		return REGISTER_ACCESS_ERROR;
	}

	uint8_t rx_data[DATA_PACKET_SIZE * NUM_MOTORS] = {0};
	uint8_t tx_data[DATA_PACKET_SIZE * NUM_MOTORS] = {0};

	int address_index = 0;
	while(address_index <= ((NUM_MOTORS - 1) * DATA_PACKET_SIZE) ){
		tx_data[address_index] = reg_addr;
		address_index += DATA_PACKET_SIZE;
	}

	// first response can be disposed of
	if(HAL_SPI_TransmitReceive(spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}

	if(HAL_SPI_TransmitReceive(spi, tx_data, rx_data, sizeof(rx_data), HAL_MAX_DELAY) != HAL_OK){
		return COMMUNICATION_ERROR;
	}else{
		int packet_index = 0;
		for(int motor_index = NUM_MOTORS - 1; motor_index >= 0; motor_index-- ){
			SPI_Status_t status;
			status.Val.Value = (rx_data[packet_index]);
			uint32_t value =  ( (rx_data[packet_index + 1] << 24) | (rx_data[packet_index + 2] << 16)| (rx_data[packet_index + 3] << 8) | rx_data[packet_index + 4] );
			packet_index += DATA_PACKET_SIZE;

			uint32_t* reg_pointer = get_reg_ptr(motor_index, reg_addr);
			*reg_pointer = value;
		}
		return TMC5160_SUCCESS;
	}

}


//////////////////////////////////////////////////////////// Utility //////////////////////////////////////

/*
 * @Brief Performs a non-blocking delay.
 * @Param delay_ms: Desired delay in milliseconds.
 * */
static void rtos_delay_p(uint32_t delay_ms){
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

static uint8_t is_readable_p(TMC5160_reg_addresses reg_addr) {

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
static uint8_t calculate_globalScaler_p(int  desired_rms_current){
	/* Current Setting:
		IRMS = (GLOBALSCALER \ 256) * (CS+1 \ 32) * (VFS / RSENSE) * (1/ sqrt(2))
		GLOBALSCALER = (IRMS * 256 * 32 * RSENSE * sqrt(2)) /  (CS+1) * VFS
	 * */
	int GS = (int)((desired_rms_current * 256 * (32 / CURRENT_SCALE + 1)  * (SENSE_RESISTOR / SENSE_VOLTAGE) *  SQRT_2));
	if(GS < 0) return 0;
	if(GS > 255) return 255;
	return (uint8_t)GS ;
}

/*
 * @Brief Converts the MRES register to base 10 micro-steps being used.
 * @Param MRES: The value of the micro-steps register.
 * */
static uint16_t tmc_mres_to_microsteps_p(uint8_t mres)
{
    return 1 << mres;
}

/*
 * @Brief Tune StealthChop for quiet motor movement.
 * @Param motor: Pointer to an instance of the motor struct.
 * */
static HAL_StatusTypeDef TMC5160_setup_stealthchop_p(TMC5160_HandleTypeDef *motor){
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
		TMC5160_move_to_position(motor, 1600);
		rtos_delay(100);
		TMC5160_move_to_position(motor, -1600);
	}

	return HAL_OK;
}

/*
 * @Brief Enables or disables the motor via the EN_PIN.
 * @Param motor: Pointer to an instance of the motor struct.
 * @Param state: desired pin state, 0 = motor on.
 * */
void TMC5160_motor_enable_p(TMC5160_HandleTypeDef *motor, GPIO_PinState state){
	HAL_GPIO_WritePin(motor->motor_en_port, motor->motor_en_pin, state);
}

/*@Brief Get the register pointer based on the offset.
 *@Param motor_index: The motor in the user made motor array you want the pointer of.
 *@Param reg_addr: The register address whose pointer you want.
 *@Return: Pointer to the register that was requested.
 * */
static uint32_t* get_reg_ptr(uint8_t motor_index, uint8_t reg_addr) {
	// TODO: the registers might be 0 by default.
    if (reg_addr > 0x7F || reg_offsets[reg_addr] == 0) {
        return NULL;
    }
    // &motors[motor_index].registers gets the base address of *this motor's* registers
    // Cast to char* to add bytes, then to uint32_t* for the .Value
    return (uint32_t*)((char*)&motors[motor_index].registers + reg_offsets[reg_addr]);
}

