/**
  ******************************************************************************
  * @file    simpleTimer.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    01/08-2022 (DD/MM-YYYY)
  * @brief   Source for simpleTimer.h file.
   TODO: Fill this module handles.....
  *******************************************************************************
  * @copy
  *
  * <h2><center>&copy; COPYRIGHT 2022</center></h2>
  *
  */

/* Includes -----------------------------------------------------------*/
#include "simpleTimer.h"

/* Defines ------------------------------------------------------------*/


/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/

static volatile uint32_t u32elapsedTicks;
static volatile uint32_t u32elapsedSeconds;
static volatile uint32_t u32currentTimeSeconds;
static volatile uint32_t u32currentTimeMilliseconds;

void simpleTimer_idle(void)
{
  u32currentTimeMilliseconds = simpleTimer_getMilliseconds();
  u32currentTimeSeconds = simpleTimer_getSeconds();
}

void simpleTimer_reset_seconds(sTimer_t* timer, uint16_t u16resetValue)
{
  if(timer->eTimerStatus != STS_RUNNING )
  {
    timer->u32timeOfReset = simpleTimer_getSeconds();
    timer->u16resetValue = u16resetValue;
    timer->bIsMillisecondsTimer = false;
    timer->eTimerStatus = STS_RUNNING;
  }
}

void simpleTimer_reset_milliSeconds(sTimer_t* timer, uint16_t u16resetValue)
{
  if(timer->eTimerStatus != STS_RUNNING )
  {
    timer->u32timeOfReset = simpleTimer_getMilliseconds();
    timer->u16resetValue = u16resetValue;
    timer->bIsMillisecondsTimer = true;
    timer->eTimerStatus = STS_RUNNING;
  }
}


bool_t simpleTimer_timeout(sTimer_t* timer)
{
  bool_t bTimedOut = false;

  if(timer->bIsMillisecondsTimer)
  {
    if(    ( (timer->u32timeOfReset + timer->u16resetValue) < u32currentTimeMilliseconds )
        && (timer->eTimerStatus == STS_RUNNING)                                             )
    {
      bTimedOut = true;
      timer->eTimerStatus = STS_TIMED_OUT;
    }
  }
  else
  {
    if(    ( (timer->u32timeOfReset + timer->u16resetValue) < u32currentTimeSeconds)
	&& (timer->eTimerStatus == STS_RUNNING)                                       )
    {
      bTimedOut = true;
      timer->eTimerStatus = STS_TIMED_OUT;
    }
  }
  return bTimedOut;
}


/* Public functions ----------------------------------------------------*/


void simpleTimer_IRQHandler(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM17)
  {
    ++u32elapsedTicks;

    if((u32elapsedTicks % 1000) == 0)
    {
      ++u32elapsedSeconds;
    }
  }
}


uint32_t simpleTimer_getSeconds(void)
{
  return u32elapsedSeconds;
}

uint32_t simpleTimer_getMilliseconds(void)
{
  return u32elapsedTicks;
}
/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2022*****END OF FILE****/
