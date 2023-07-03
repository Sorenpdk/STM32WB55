/*
 * SP_IntFlash.h
 *
 *  Created on: Jul 3, 2023
 *      Author: soren
 */

#ifndef INC_SP_INTFLASH_H_
#define INC_SP_INTFLASH_H_
#include "common.h"
#include "stm32wb55xx.h"

bool_t drv_IntFlash_checkBusy(void);
bool_t drv_IntFlash_checkOperationsAllowed(void);
void drv_IntFlash_Unlock(void);
bool_t drv_IntFlash_Write(void);

#endif /* INC_SP_INTFLASH_H_ */
