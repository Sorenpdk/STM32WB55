/**
  ******************************************************************************
  * @file    bitSet.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    13/06-2023 (DD/MM-YYYY)
  * @brief   Source for bitSet.h file.
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
#include "bitSet.h"

/* Defines ------------------------------------------------------------*/


/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/
bool_t bitSet_isFlagPositionSet(Bitset_t* bitset, uint8_t u8position)
{
  return (bitset->au8Bitset[0] & u8position);
}

void bitSet_setBit(Bitset_t* bitset, uint8_t u8position)
{
  uint8_t u8bitsetIndex = u8position / 8;
  uint8_t u8bitPosition = u8position % 8;

  bitset->au8Bitset[u8bitsetIndex] |= (1 << u8bitPosition);
}

void bitSet_clearBit(Bitset_t* bitset, uint8_t u8position)
{
  uint8_t u8bitsetIndex = u8position / 8;
  uint8_t u8bitPosition = u8position % 8;

  bitset->au8Bitset[u8bitsetIndex] &= ~(1 << u8bitPosition);
}

bool_t bitSet_anyBitset(Bitset_t* bitset)
{
  for (uint8_t u8idx = 0; u8idx < BIT_SET_SIZE_BYTES; u8idx++)
  {
      if (bitset[u8idx].au8Bitset[0] != 0)
      {
	return TRUE; /* Short circuit here */
      }
  }
  return FALSE;
}

uint8_t bitSet_numberOfBitsSet(Bitset_t* bitset)
{
  uint8_t u8count = 0;
  for (uint8_t u8idx = 0; u8idx < BIT_SET_SIZE_BYTES; u8idx++)
  {
      u8count += __builtin_popcount(bitset->au8Bitset[u8idx]);
  }
  return u8count;
}

/* __builtin_ctz perform the operation in a single machine instruction */
int8_t bitSet_findFirstBit(Bitset_t* bitset)
{
  for (uint8_t u8idx = 0; u8idx < BIT_SET_SIZE_BYTES; u8idx++)
  {
      if (bitset->au8Bitset[u8idx] != 0)
	  return ( (u8idx * BITS_PER_BYTE) + __builtin_ctz(bitset->au8Bitset[u8idx]));
  }
  return -1; // Indicate no set bit found
}


/* Private functions ---------------------------------------------------*/


/******************* (C) COPYRIGHT 2023*****END OF FILE****/
