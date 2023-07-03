/**
  ******************************************************************************
  * @file    SP_SPI.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    26/06-2023 (DD/MM-YYYY)
  * @brief   Header for SP_SPI.c file.
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
#ifndef __SPSPI_H
#define __SPSPI_H

/* Private define ------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32wb55xx.h"
#include "stm32wbxx_it.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void drv_SPI_init(void);
void drv_SPI_idle(void);
void drv_SPI_transmit_nBytes(uint8_t* pu8Data, uint16_t u16dataLength);
void drv_SPI_transmit(uint8_t u8data);
void drv_SPI_IRQHandler(void);

#endif /* __SPSPI_H */

/******************* (C) COPYRIGHT *****END OF FILE****/


