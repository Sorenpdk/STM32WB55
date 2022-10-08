/**
  ******************************************************************************
  * @file    app_main.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    08/10-2022 (DD/MM-YYYY)
  * @brief   Source for app_main.h file.
   TODO: Fill this module handles.....
  *******************************************************************************
  * @copy
  MIT License

  Copyright (c) 2022 Søren Pørksen

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/* Includes -----------------------------------------------------------*/
#include "app_main.h"

#include "simpleTimer.h"
#include "ringBuffer.h"
/* Defines ------------------------------------------------------------*/
#define BUFFER_SIZE 16

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
void app_main_init(void)
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
static uint8_t u8dex;
void app_main_idle(void)
{
  simpleTimer_idle();

  if(simpleTimer_timeout(&ledONTimer))
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    simpleTimer_reset_milliSeconds(&ledOFFTimer, OFFTime);
    //u8dex = ringBuffer_get(&ringbuf);
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
