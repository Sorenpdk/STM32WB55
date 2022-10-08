/**
  ******************************************************************************
  * @file    ringBuffer.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    TODO: fill 01/08-2022 (DD/MM-YYYY)
  * @brief   Header for ringBuffer.c file.
    TODO: fill this module handles....
  *******************************************************************************
  * @copy
  *
  * <h2><center>&copy; COPYRIGHT 2022 </center></h2>
  *
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RING_BUFFER_H
#define __RING_BUFFER_H

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  RS_EMPTY = 0,
  RS_HAS_ELEMENTS,
  RS_NOT_ENOUGH_CAPACITY,
  RS_FULL

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
void ringBuffer_idle(void);
bool_t ringBuffer_put(sRingbuf_t* pRingBuffer, uint8_t u8value);
uint8_t ringBuffer_get(sRingbuf_t* pRingBuffer);

#endif /* __RING_BUFFER_H */

/******************* (C) COPYRIGHT *****END OF FILE****/


