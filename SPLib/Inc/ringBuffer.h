/**
  ******************************************************************************
  * @file    ringBuffer.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    08/10-2022 (DD/MM-YYYY)
  * @brief   Header for ringBuffer.c file.
    TODO: fill this module handles....
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  RINGBUF_EMPTY = 0,
  RINGBUF_HAS_ELEMENTS,
  RINGBUF_FULL

}eRingbufStatus_t;


typedef struct
{
  uint8_t* au8buffer;
  uint16_t u16putIndex;
  uint16_t u16getIndex;
  uint16_t u16elementsInBuffer;
  uint16_t u16bufferMaxCapacity;
  eRingbufStatus_t eRingBufStatus;

}sRingbuf_t;

/* Exported constants --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void ringBuffer_init(sRingbuf_t* pRingBuffer, uint8_t* pu8buffer, uint16_t u16bufferSizeBytes);
bool_t ringBuffer_put(sRingbuf_t* pRingBuffer, uint8_t u8value);
uint8_t ringBuffer_get(sRingbuf_t* pRingBuffer);
uint8_t ringBuffer_peek(sRingbuf_t* pRingBuffer, uint16_t u16index);
#endif /* __RING_BUFFER_H */

/******************* (C) COPYRIGHT *****END OF FILE****/


