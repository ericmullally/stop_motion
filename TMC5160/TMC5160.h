#ifndef __TMC5160_H
#define __TMC5160_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "TMC5160_Registers.h"
#include "TMC5160_Structs.h"


#define WRITE_MASK 0x80


HAL_StatusTypeDef TMC5160_init(TMC5160_HandleTypeDef *motor);
HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data);
SPI_Status_t TMC5160_ReadRegister(TMC5160_HandleTypeDef *motor, uint32_t * reg_val, uint8_t reg_addr);
HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor);
void TMC5160_set_max_velocity(TMC5160_HandleTypeDef *motor, int velocity);
void TMC5160_set_max_acceleration(TMC5160_HandleTypeDef *motor, int acceleration);
void TMC5160_move_to_position(TMC5160_HandleTypeDef *motor, int full_steps);


#ifdef __cplusplus
}
#endif

#endif /* __TMC5160_H */
