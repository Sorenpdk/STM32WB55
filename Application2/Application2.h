/**
  ******************************************************************************
  * @file    Application.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    TODO: fill 01/08-2022 (DD/MM-YYYY)
  * @brief   Header for WakeUpMsgQueue.c file.
    TODO: fill this module handles....
  *******************************************************************************
  * @copy
  *
  * <h2><center>&copy; COPYRIGHT 2022 </center></h2>
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APPLICATION_H
#define __APPLICATION_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdlib.h"
#include "main.h"
#include "stm32wbxx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void Application_init(void);
void Application_idle(void);

#endif /* __APPLICATION_H */

/******************* (C) COPYRIGHT *****END OF FILE****/


