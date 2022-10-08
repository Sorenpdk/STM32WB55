/**
  ******************************************************************************
  * @file    ringBuffer.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    01/08-2022 (DD/MM-YYYY)
  * @brief   Source for ringBuffer.h file.
   TODO: Fill this module handles.....
  *******************************************************************************
  * @copy
  *
  * <h2><center>&copy; COPYRIGHT 2022</center></h2>
  *
  */

/* Includes -----------------------------------------------------------*/
#include "ringBuffer.h"

/* Defines ------------------------------------------------------------*/


/* Private function prototypes ----------------------------------------*/
static inline uint16_t buffer_capacity_available(sRingbuf_t* pRingBuffer);

/* Global variables ---------------------------------------------------*/

void ringBuffer_init(sRingbuf_t* pRingBuffer, uint8_t* pu8buffer, uint16_t u16bufferSizeBytes)
{
  pRingBuffer->au8buffer = &pu8buffer[0];
  pRingBuffer->u16bufferMaxCapacity = u16bufferSizeBytes;
  pRingBuffer->eRingBufStatus = RS_EMPTY;
  pRingBuffer->u16elementsInBuffer = 0;
  pRingBuffer->u16getIndex = 0;
  pRingBuffer->u16putIndex = 0;
}

void ringBuffer_idle(void)
{

}


/* Public functions ----------------------------------------------------*/
bool_t ringBuffer_put(sRingbuf_t* pRingBuffer, uint8_t u8value)
{
  bool_t bStatus = false;

  if(buffer_capacity_available(pRingBuffer) != 0)
  {
    pRingBuffer->au8buffer[pRingBuffer->u16putIndex++] = u8value;
    pRingBuffer->u16elementsInBuffer++;
	bStatus = true;
	if(pRingBuffer->eRingBufStatus == RS_EMPTY)
	{
		pRingBuffer->eRingBufStatus = RS_HAS_ELEMENTS;
	}
  }
  return bStatus;
}


uint8_t ringBuffer_get(sRingbuf_t* pRingBuffer)
{
  uint8_t u8returnValue = 0;

  if(pRingBuffer->eRingBufStatus != RS_EMPTY)
  {
	u8returnValue = pRingBuffer->au8buffer[pRingBuffer->u16getIndex++];
	pRingBuffer->au8buffer[pRingBuffer->u16getIndex-1] = 0;
	pRingBuffer->u16elementsInBuffer--;
  }

  return u8returnValue;
}


/* Private functions ---------------------------------------------------*/
static inline uint16_t buffer_capacity_available(sRingbuf_t* pRingBuffer)
{
  return ( pRingBuffer->u16bufferMaxCapacity - pRingBuffer->u16elementsInBuffer );
}

/******************* (C) COPYRIGHT 2022*****END OF FILE****/
