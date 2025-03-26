#ifndef __TMC5160_H
#define __TMC5160_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "TMC5160_Registers.h"
#include "TMC5160_Structs.h"


#define WRITE_MASK 0x80

HAL_StatusTypeDef TMC5160_WriteRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr, uint32_t data);
uint32_t TMC5160_ReadRegister(TMC5160_HandleTypeDef *motor, uint8_t reg_addr);
HAL_StatusTypeDef TMC5160_setup_stealthchop(TMC5160_HandleTypeDef *motor);





#ifdef __cplusplus
}
#endif

#endif /* __TMC5160_H */
