/**
  ******************************************************************************
  * @file    bitSet.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    13/06-2023 (DD/MM-YYYY)
  * @brief   Header for bitSet.c file.
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
#ifndef __BITSET_H
#define __BITSET_H

/* Private define ------------------------------------------------------------*/
#define BIT_SET_SIZE_BYTES   2

/* Includes ------------------------------------------------------------------*/
#include "common.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
    uint8_t au8Bitset[BIT_SET_SIZE_BYTES];

} Bitset_t;
/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
bool_t bitSet_isFlagPositionSet(Bitset_t* bitset, uint8_t u8position);
bool_t bitSet_anyBitset(Bitset_t* bitset);
uint8_t bitSet_numberOfBitsSet(Bitset_t* bitset);
int8_t bitSet_findFirstBit(Bitset_t* bitset);
void bitSet_setBit(Bitset_t* bitset, uint8_t u8position);
void bitSet_clearBit(Bitset_t* bitset, uint8_t u8position);


#endif /* __BITSET_H */

/******************* (C) COPYRIGHT *****END OF FILE****/

