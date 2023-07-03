/**
  ******************************************************************************
  * @file    SP_GPIO.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    21/06-2023 (DD/MM-YYYY)
  * @brief   Source for SP_GPIO.h file.
   TODO: Fill this module handles.....
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

/* Includes -----------------------------------------------------------*/
#include "SP_GPIO.h"

/* Defines ------------------------------------------------------------*/
/* Private function prototypes ----------------------------------------*/
static void set_GPIO_mode(GPIO_init_t* pGPIOMode, GPIO_TypeDef* GPIOx);
static void set_GPIO_type(GPIO_init_t* pGPIOType, GPIO_TypeDef* GPIOx);
static void set_GPIO_speed(GPIO_init_t* pGPIOSpeed, GPIO_TypeDef* GPIOx);
static void set_GPIO_pull(GPIO_init_t* pGPIOPull, GPIO_TypeDef* GPIOx);

static GPIO_TypeDef* get_GPIO_port(GPIO_init_t* pGPIOInit);

/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/

void drv_GPIO_init(GPIO_init_t* pGPIOInit)
{
  GPIO_TypeDef* GPIOx;

  GPIOx = get_GPIO_port(pGPIOInit);
  set_GPIO_mode(pGPIOInit, GPIOx);
  set_GPIO_type(pGPIOInit, GPIOx);
  set_GPIO_speed(pGPIOInit, GPIOx);
  set_GPIO_pull(pGPIOInit, GPIOx);

}

void drv_GPIO_set_pin(eGPIO_Ports_t ePort, bool_t bState, uint8_t u8pinNumber)
{
  switch(ePort)
  {
    case GPIO_PORTA: { bState ? (GPIOA->ODR |= (0x01 << u8pinNumber) ) : (GPIOA->ODR &= ~(0x01 << u8pinNumber) ); } break;
    case GPIO_PORTB: { bState ? (GPIOB->ODR |= (0x01 << u8pinNumber) ) : (GPIOB->ODR &= ~(0x01 << u8pinNumber) ); } break;
    case GPIO_PORTC: { bState ? (GPIOC->ODR |= (0x01 << u8pinNumber) ) : (GPIOC->ODR &= ~(0x01 << u8pinNumber) ); } break;
    default : break;
  }
}


/* Private functions ---------------------------------------------------*/
static void set_GPIO_pull(GPIO_init_t* pGPIOPull, GPIO_TypeDef* GPIOx)
{
  switch(pGPIOPull->gpioPuPd)
   {
     case GPIO_NO_PULL:   { GPIOx->PUPDR &= ~(0x01 << (pGPIOPull->u8pinNumber * 2)  ); } break;
     case GPIO_PULLUP:    { GPIOx->PUPDR |=  (0x01 << (pGPIOPull->u8pinNumber * 2) ); } break;
     case GPIO_PULL_DOWN: { GPIOx->PUPDR |=  (0x02 << (pGPIOPull->u8pinNumber * 2) ); } break;
     default: break;
   }
}


static void set_GPIO_speed(GPIO_init_t* pGPIOSpeed, GPIO_TypeDef* GPIOx)
{
  switch(pGPIOSpeed->gpioSpeed)
   {
     case GPIO_SPEED_LOW:    { GPIOx->OSPEEDR &= ~(0x01 << (pGPIOSpeed->u8pinNumber * 2) ); } break;
     case GPIO_SPEED_MEDIUM: { GPIOx->OSPEEDR |=  (0x01 << (pGPIOSpeed->u8pinNumber * 2) ); } break;
     case GPIO_SPEED_FAST:   { GPIOx->OSPEEDR |=  (0x02 << (pGPIOSpeed->u8pinNumber * 2) ); } break;
     case GPIO_SPEED_HIGH:   { GPIOx->OSPEEDR |=  (0x03 << (pGPIOSpeed->u8pinNumber * 2) ); } break;
     default: break;
   }
}


static void set_GPIO_type(GPIO_init_t* pGPIOType, GPIO_TypeDef* GPIOx)
{
  switch(pGPIOType->gpioType)
   {
     case GPIO_TYPE_PUSH_PULL:  { GPIOx->OTYPER &=  ~(0x01 << pGPIOType->u8pinNumber ); } break;
     case GPIO_TYPE_OPEN_DRAIN: { GPIOx->OTYPER |=   (0x01 << pGPIOType->u8pinNumber ); } break;

     default: break;
   }
}


static GPIO_TypeDef* get_GPIO_port(GPIO_init_t* pGPIOInit)
{
  GPIO_TypeDef* pretVal = NULL;

  switch(pGPIOInit->gpioPort)
  {
    case GPIO_PORTA : { pretVal = GPIOA; } break;
    case GPIO_PORTB : { pretVal = GPIOB; } break;
    case GPIO_PORTC : { pretVal = GPIOC; } break;

    default: break;
  }

  return pretVal;
}

static void set_GPIO_mode(GPIO_init_t* pGPIOMode, GPIO_TypeDef* GPIOx)
{
  switch(pGPIOMode->gpioMode)
  {
     case GPIO_MODE_INPUT  : {  GPIOx->MODER &= ~(0x01 << ((pGPIOMode->u8pinNumber * 2))); } break;
     case GPIO_MODER_OUTPUT :
     {
       GPIOx->MODER &= ~(0x03 << ((pGPIOMode->u8pinNumber * 2))); /* Clear the reset value */
       GPIOx->MODER |=  (0x01 << ((pGPIOMode->u8pinNumber * 2)));
     } break;
     case GPIO_MODER_ALTERNATE 	:
       {
	 GPIOx->MODER &= ~(0x03 << ((pGPIOMode->u8pinNumber * 2))); /* Clear the reset value */
	 GPIOx->MODER |=  (0x02 << ((pGPIOMode->u8pinNumber * 2)));
       } break;
     case GPIO_MODER_ANALOG 	: { } break;

     default: break;
   }
}

/******************* (C) COPYRIGHT 2023*****END OF FILE****/
