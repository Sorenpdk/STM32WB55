/**
  ******************************************************************************
  * @file    SP_Uart.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    18/06-2023 (DD/MM-YYYY)
  * @brief   Header for SP_Uart.c file.
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
#ifndef __SPUART_H
#define __SPUART_H

/* Private define ------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32wb55xx.h"
#include "stm32wbxx_it.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  UART_BAUD_9600 = 0,
  UART_BAUD_38400,
  UART_BAUD_115200

} eBaudrate_t;

typedef enum
{
  UART_STOP_BITS_1 = 0,
  UART_STOP_BITS_2

} eStopBits_t;

typedef enum
{
  UART_PARITY_NONE = 0,
  UART_PARITY_EVEN,
  UART_PARITY_ODD

} eParity_t;

typedef enum
{
  UART_WORD_LEGTH_8_BITS = 0,
  UART_WORD_LEGTH_7_BITS,
  UART_WORD_LEGTH_9_BITS,

} eWordLength_t;

typedef struct
{
  eBaudrate_t uartBaud;
  eStopBits_t uartStopBits;
  eParity_t uartParity;
  eWordLength_t uartWordLength;

} UART_init_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void drv_uart_init(UART_init_t* pUARTInit);
void drv_uart_idle(void);
void drv_uart_trasnmit(uint8_t *pu8Data, uint16_t u16DataLengthBytes);
void drv_uart_receive(uint8_t *pu8Data, uint16_t u16DataLengthBytes);
void drv_uart_IRQHandler(void);
void drv_uart_transmit(void);
#endif /* __SPUART_H */

/******************* (C) COPYRIGHT *****END OF FILE****/
