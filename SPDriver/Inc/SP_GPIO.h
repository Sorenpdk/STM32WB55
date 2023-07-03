/**
  ******************************************************************************
  * @file    SP_GPIO.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    21/06-2023 (DD/MM-YYYY)
  * @brief   Header for SP_GPIO.c file.
    TODO: fill this module handles....
  *******************************************************************************
  * @copy
  MIT License

  Copyright (c) 2023 Søren Pørksen

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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPGPIO_H
#define __SPGPIO_H

/* Private define ------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32wb55xx.h"
#include "stm32wbxx_it.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  GPIO_PORTA = 0,
  GPIO_PORTB,
  GPIO_PORTC

} eGPIO_Ports_t;


typedef enum
{
  GPIO_MODE_INPUT = 0,
  GPIO_MODER_OUTPUT,
  GPIO_MODER_ALTERNATE,
  GPIO_MODER_ANALOG

} eGPIO_Mode_t;

typedef enum
{
  GPIO_TYPE_PUSH_PULL = 0,
  GPIO_TYPE_OPEN_DRAIN

} eGPIO_Type_t;

typedef enum
{
  GPIO_SPEED_LOW = 0,
  GPIO_SPEED_MEDIUM,
  GPIO_SPEED_FAST,
  GPIO_SPEED_HIGH

} eGPIO_Speed_t;

typedef enum
{
  GPIO_NO_PULL = 0,
  GPIO_PULLUP,
  GPIO_PULL_DOWN

} eGPIO_PuPd_t;


typedef struct
{
  eGPIO_Ports_t gpioPort;
  eGPIO_Mode_t gpioMode;
  eGPIO_Type_t gpioType;
  eGPIO_Speed_t gpioSpeed;
  eGPIO_PuPd_t gpioPuPd;
  uint8_t u8pinNumber;

} GPIO_init_t;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void drv_GPIO_init(GPIO_init_t* pGPIOInit);
void drv_GPIO_set_pin(eGPIO_Ports_t ePort, bool_t bState, uint8_t u8pinNumber);

#endif /* __SPGPIO_H */

/******************* (C) COPYRIGHT *****END OF FILE****/
