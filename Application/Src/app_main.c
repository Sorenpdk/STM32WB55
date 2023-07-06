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
#include "simpleFSM.h"
#include "bitSet.h"
#include "linkedList.h"
#include <time.h>
#include "SP_GPIO.h"
#include "SP_SPI.h"
#include "SP_Uart.h"
#include "SP_IntFlash.h"
#include "accelerometer_lis2dw12.h"
/* Defines ------------------------------------------------------------*/
#define BUFFER_SIZE 16

/* Private function prototypes ----------------------------------------*/
static void test(void);
static void bitset_Test(void);

void entry_state_handler();
void action_state_handler(int* newState);
void exit_state_handler();

void entry_state_handler2();
void action_state_handler2(int* newState);
void exit_state_handler2();

void entry_state_handler3();
void action_state_handler3(int* newState);
void exit_state_handler3();

static inline uint32_t DWT_us(void);
/* Global variables ---------------------------------------------------*/
static sTimer_t ledONTimer;
static sTimer_t ledOFFTimer;
static sTimer_t testTimer;
static uint16_t ONTime = 100;
static uint16_t OFFTime = 100;
static Bitset_t bitset;
static sRingbuf_t ringbuf;
static uint8_t au8buffer[BUFFER_SIZE];

static fsm_t myfsm;

static const state_table_t states[] =
{
  {First, entry_state_handler, action_state_handler, exit_state_handler},
  {Second, entry_state_handler2, action_state_handler2, exit_state_handler2},
  {Third, entry_state_handler3, action_state_handler3, exit_state_handler3},
};


uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}


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
  ringBuffer_init(&ringbuf, au8buffer, BUFFER_SIZE);
  simple_fsm_init(&myfsm, sizeof(states)/sizeof(states[0]), states);

  DWT_Delay_Init();
  SystemCoreClockUpdate();

  UART_init_t UART1_Init = { .uartBaud = UART_BAUD_115200,
			    .uartParity = UART_PARITY_NONE,
			    .uartStopBits = UART_STOP_BITS_1,
			    .uartWordLength = UART_WORD_LEGTH_8_BITS };




  drv_uart_init(&UART1_Init);

  drv_SPI_init();

  simpleTimer_reset_milliSeconds(&testTimer, ONTime);
  uwTickPrio = TICK_INT_PRIORITY;

  acc_init();
}

uint8_t ascendingValues[100] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
    70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
    90, 91, 92, 93, 94, 95, 96, 97, 98, 99
};


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

static bool_t btestgpio;
static uint32_t u32test;
static uint32_t u32test2;
static uint32_t u32testclock;
static uint32_t u32previousUsTime;
static uint32_t u32nowUsTime;

static unsigned char acmsg[] = "Hello from STM";

void app_main_idle(void)
{
  u8idx++;
  acc_getTemperature();

  HAL_Delay(200);
}

/* CYCCNT increments on each cycle of the processor clock */
static inline uint32_t DWT_us(void)
{
  return DWT->CYCCNT / 64;
}

/* Public functions ----------------------------------------------------*/




void entry_state_handler()
{

}

void action_state_handler(int* newState)
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  if(simpleTimer_timeout(&ledONTimer))
  {
    simpleTimer_reset_milliSeconds(&ledOFFTimer, OFFTime);
    *newState = Second;
  }
}

void exit_state_handler()
{

}

void entry_state_handler2()
{

}

void action_state_handler2(int* newState)
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);

  if(simpleTimer_timeout(&ledOFFTimer))
   {
     simpleTimer_reset_milliSeconds(&ledONTimer, ONTime);
     *newState = Third;
   }

}

void exit_state_handler2()
{

}

void entry_state_handler3()
{

}

void action_state_handler3(int* newState)
{

    *newState = First;
}

void exit_state_handler3()
{

}



/* Private functions ---------------------------------------------------*/
static void test(void)
{
  simpleTimer_idle();

  if(simpleTimer_timeout(&ledONTimer))
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    simpleTimer_reset_milliSeconds(&ledOFFTimer, OFFTime);
    u8dex = ringBuffer_get(&ringbuf);
  }

  if(simpleTimer_timeout(&ledOFFTimer))
  {
    simpleTimer_reset_milliSeconds(&ledONTimer, ONTime);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    ringBuffer_put(&ringbuf, u8idx++);
  }
}


static uint8_t bitset_x;
static int8_t bitset_y;
static void bitset_Test(void)
{
  bitSet_setBit(&bitset, 0);
  bitSet_setBit(&bitset, 1);
  bitSet_setBit(&bitset, 2);
  bitSet_setBit(&bitset, 3);
  bitSet_setBit(&bitset, 4);
  bitSet_setBit(&bitset, 5);
  bitSet_setBit(&bitset, 6);
  bitSet_setBit(&bitset, 7);

  bitset_x = bitSet_numberOfBitsSet(&bitset);

  bitSet_clearBit(&bitset, 0);
  bitSet_clearBit(&bitset, 1);
  bitSet_clearBit(&bitset, 2);
  bitSet_clearBit(&bitset, 3);
  bitSet_clearBit(&bitset, 4);
  bitSet_clearBit(&bitset, 5);
  bitSet_clearBit(&bitset, 6);
  bitSet_clearBit(&bitset, 7);

  bitset_x = bitSet_numberOfBitsSet(&bitset);

  bitSet_setBit(&bitset, 8);

  bitset_y = bitSet_findFirstBit(&bitset);
}
/******************* (C) COPYRIGHT 2022*****END OF FILE****/
