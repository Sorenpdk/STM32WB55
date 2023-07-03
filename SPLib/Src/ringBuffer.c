/**
  ******************************************************************************
  * @file    ringBuffer.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    08/10-2022 (DD/MM-YYYY)
  * @brief   Source for ringBuffer.h file.
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
#include "ringBuffer.h"

/* Defines ------------------------------------------------------------*/


/* Private function prototypes ----------------------------------------*/
static inline bool_t isRingBuffer_full(sRingbuf_t* pRingBuffer);

/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/
void ringBuffer_init(sRingbuf_t* pRingBuffer, uint8_t* pu8buffer, uint16_t u16bufferSizeBytes)
{
  pRingBuffer->au8buffer 	    = &pu8buffer[0];
  pRingBuffer->u16bufferMaxCapacity = u16bufferSizeBytes;
  pRingBuffer->u16elementsInBuffer  = 0;
  pRingBuffer->u16getIndex  	    = 0;
  pRingBuffer->u16putIndex          = 0;
  pRingBuffer->eRingBufStatus       = RINGBUF_EMPTY;
}

bool_t ringBuffer_put(sRingbuf_t* pRingBuffer, uint8_t u8value)
{
  __disable_irq();

  bool_t bStatus = false;

  if( !isRingBuffer_full(pRingBuffer))
  {
    pRingBuffer->au8buffer[pRingBuffer->u16putIndex] = u8value;
    pRingBuffer->u16putIndex = (pRingBuffer->u16putIndex + 1) % (pRingBuffer->u16bufferMaxCapacity);
    pRingBuffer->u16elementsInBuffer++;
    bStatus = true;
  }
  __enable_irq();
  return bStatus;
}


uint8_t ringBuffer_get(sRingbuf_t* pRingBuffer)
{
  __disable_irq();

  uint8_t u8returnValue = 0;

  if(    (pRingBuffer->eRingBufStatus != RINGBUF_EMPTY        )
      && (pRingBuffer->u16getIndex != pRingBuffer->u16putIndex)  )
  {
    u8returnValue = pRingBuffer->au8buffer[pRingBuffer->u16getIndex];
    pRingBuffer->au8buffer[pRingBuffer->u16getIndex] = 0;
    pRingBuffer->u16elementsInBuffer--;
    pRingBuffer->u16getIndex = (pRingBuffer->u16getIndex + 1) % (pRingBuffer->u16bufferMaxCapacity);

  }
  __enable_irq();
  return u8returnValue;
}

uint8_t ringBuffer_peek(sRingbuf_t* pRingBuffer, uint16_t u16index)
{
  return pRingBuffer->au8buffer[u16index];
}

/* Private functions ---------------------------------------------------*/
static inline bool_t isRingBuffer_full(sRingbuf_t* pRingBuffer)
{
  bool_t bStatus = false;

  bStatus = ( (pRingBuffer->u16elementsInBuffer == pRingBuffer->u16bufferMaxCapacity) ? true : false );

  if(bStatus)
  {
    pRingBuffer->eRingBufStatus = RINGBUF_FULL;
  }
  else if(pRingBuffer->u16elementsInBuffer)
  {
    pRingBuffer->eRingBufStatus = RINGBUF_HAS_ELEMENTS;
  }
  else
  {
    pRingBuffer->eRingBufStatus = RINGBUF_EMPTY;
  }
  return bStatus;
}

/******************* (C) COPYRIGHT 2022*****END OF FILE****/
