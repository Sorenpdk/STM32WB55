/**
  ******************************************************************************
  * @file    SP_DMA.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    3/03-2024 (DD/MM-YYYY)
  * @brief   Header for SP_DMA.c file.
    TODO: fill this module handles....
  *******************************************************************************
  * @copy
  MIT License

  Copyright (c) 2024 Søren Pørksen

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
#ifndef __SPDMA_H
#define __SPDMA_H

/* Private define ------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "stm32wb55xx.h"
#include "stm32wbxx_it.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void drv_DMA_init(void);
void DMA_Configuration(uint8_t* data, uint16_t size);
void DMA_IRQHandler(void) ;
#endif /* __SPDMA_H */

/******************* (C) COPYRIGHT *****END OF FILE****/
