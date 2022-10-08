/**
  ******************************************************************************
  * @file    Application.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    01/08-2022 (DD/MM-YYYY)
  * @brief   Source for Application.h file.
   TODO: Fill this module handles.....
  *******************************************************************************
  * @copy
  *
  * <h2><center>&copy; COPYRIGHT 2022</center></h2>
  *
  */

/* Includes -----------------------------------------------------------*/
#include "Application2.h"
#include "simpleTimer.h"
#include "ringBuffer.h"
/* Defines ------------------------------------------------------------*/
#define BUFFER_SIZE 128

/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/
static sTimer_t ledONTimer;
static sTimer_t ledOFFTimer;
static uint16_t ONTime = 500;
static uint16_t OFFTime = 100;

static sRingbuf_t ringbuf;
static uint8_t au8buffer[BUFFER_SIZE];

/*---------------------------------------------------------------------*/
/**
  * @brief
  * Initialization of the
  * @param  none
  * @retval none
  */
void Application_init(void)
{
  simpleTimer_reset_milliSeconds(&ledONTimer, ONTime);
  simpleTimer_reset_milliSeconds(&ledOFFTimer, OFFTime);
  ringBuffer_init(&ringbuf, au8buffer, BUFFER_SIZE);
}


/*----------------------------------------------------------------------*/
/**
  * @brief
  *
  *
  * @param  none
  * @retval none
  */
static uint8_t u8idx;
static uint8_t u8x;
void Application_idle(void)
{
  simpleTimer_idle();

  if(simpleTimer_timeout(&ledONTimer))
  {

	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  simpleTimer_reset_milliSeconds(&ledOFFTimer, OFFTime);
	  u8x = ringBuffer_get(&ringbuf);
  }

  if(simpleTimer_timeout(&ledOFFTimer))
   {
	  simpleTimer_reset_milliSeconds(&ledONTimer, ONTime);
 	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 	  ringBuffer_put(&ringbuf, u8idx++);
   }

}

/* Public functions ----------------------------------------------------*/


/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2022*****END OF FILE****/
