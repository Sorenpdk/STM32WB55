/**
  ******************************************************************************
  * @file    simpleTimer.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    TODO: fill 01/08-2022 (DD/MM-YYYY)
  * @brief   Header for simpleTimer.c file.
    TODO: fill this module handles....
  *******************************************************************************
  * @copy
  *
  * <h2><center>&copy; COPYRIGHT 2022 </center></h2>
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SIMPLE_TIMER_H
#define __SIMPLE_TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32wbxx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
 STS_STOPPED = 0,
 STS_RUNNING,
 STS_TIMED_OUT

}eSimpleTimerStatus_t;


typedef struct
{
 uint32_t u32timeOfReset;
 eSimpleTimerStatus_t eTimerStatus;
 uint16_t u16resetValue;
 bool_t bIsMillisecondsTimer;
 /* TODO: callback on timeout */
}sTimer_t;

/* Exported constants --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void simpleTimer_init(void);
void simpleTimer_idle(void);
void simpleTimer_IRQHandler(TIM_HandleTypeDef *htim);
void simpleTimer_reset_seconds(sTimer_t* timer, uint16_t u16resetValue);
void simpleTimer_reset_milliSeconds(sTimer_t* timer, uint16_t u16resetValue);
bool_t simpleTimer_timeout(sTimer_t* timer);
uint32_t simpleTimer_getSeconds(void);
uint32_t simpleTimer_getMilliseconds(void);

#endif /* __SIMPLE_TIMER_H */

/******************* (C) COPYRIGHT *****END OF FILE****/


