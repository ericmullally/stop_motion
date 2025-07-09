#ifndef __TMC5160_H
#define __TMC5160_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "TMC5160_Registers.h"
#include "TMC5160_Structs.h"


#define WRITE_MASK 0x80

typedef enum{
	ms_full = 0,
	ms_2    = 1,
	ms_4    = 2,
	ms_8    = 3 ,
	ms_16   = 4,
	ms_32   = 5,
	ms_54   = 6,
	ms_128  = 7
}TMC5160_MICROSTEPS_T;

typedef enum{
	TMC5160_SUCCESS 		= 0x00,
	NULL_MOTOR_ERROR 		= 0x01,
	REGISTER_ACCESS_ERROR 	= 0x02,
	COMMUNICATION_ERROR 	= 0x03,
	DRIVER_ERROR			= 0x04,
	STALL_ERROR				= 0x05,
	UNDERVOLTAGE_ERROR		= 0x06,
	POSITION_NOT_REACHED	= 0x07,
	MOTOR_IN_USE			= 0x08,
	RESET_FAILURE			= 0x09
}TMC5160_return_values_t;

typedef enum{
	FORWARD = 1,
	REVERSE = 2
}TMC5160_motor_directions;


typedef struct{
	uint32_t error_flag;
	TMC5160_return_values_t error;
}TMC5160_error_indicator;








TMC5160_return_values_t TMC5160_init(TMC5160_HandleTypeDef *motor);
TMC5160_return_values_t TMC5160_WriteRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data);
TMC5160_return_values_t TMC5160_ReadRegisterUnsigned(TMC5160_HandleTypeDef *motor, uint32_t * reg_val, uint8_t reg_addr);
TMC5160_return_values_t TMC5160_ReadRegisterSigned(TMC5160_HandleTypeDef *motor, int32_t * reg_val, uint8_t reg_addr);
TMC5160_return_values_t TMC5160_set_zero(TMC5160_HandleTypeDef *motor);
void TMC5160_set_max_velocity(TMC5160_HandleTypeDef *motor, int velocity);
void TMC5160_move_to_position(TMC5160_HandleTypeDef *motor, int full_steps);
void TMC5160_set_positioning_mode(TMC5160_HandleTypeDef *motor);
void TMC5160_motor_enable(TMC5160_HandleTypeDef *motor, GPIO_PinState state);
uint8_t TMC5160_get_status(TMC5160_HandleTypeDef *motor);




#ifdef __cplusplus
}
#endif

#endif /* __TMC5160_H */
