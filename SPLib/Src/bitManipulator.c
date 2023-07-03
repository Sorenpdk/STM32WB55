/**
  ******************************************************************************
  * @file    bitManipulator.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    18/06-2023 (DD/MM-YYYY)
  * @brief   Source for bitManipulator.h file.
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
#include "bitManipulator.h"

/* Defines ------------------------------------------------------------*/
/* Private function prototypes ----------------------------------------*/
/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/

uint8_t bit_convert_u32_to_u8(BitEdianess_t edianess, uint32_t u32source, uint8_t *u8output)
{
  uint8_t u8bytesHandled = 0;

  if(edianess == LITTLE_ENDIAN)
  {
    u8output[0] = (u32source & 0xFF);
    u8output[1] = ((u32source >> 24) & 0xFF);
    u8output[2] = ((u32source >> 16) & 0xFF);
    u8output[3] = ((u32source >> 8) & 0xFF);

    u8bytesHandled = sizeof(uint32_t);
  }
  else
  {
    u8output[0] = ((u32source >> 24) & 0xFF);
    u8output[1] = ((u32source >> 16) & 0xFF);
    u8output[2] = ((u32source >> 8) & 0xFF);
    u8output[3] = (u32source & 0xFF);

    u8bytesHandled = sizeof(uint32_t);
  }

  return u8bytesHandled;
}

uint8_t bit_convert_u16_to_u8(BitEdianess_t edianess, uint16_t u16source, uint8_t *u8output)
{
  uint8_t u8bytesHandled = 0;

  if(edianess == LITTLE_ENDIAN)
  {
    u8output[0] = (u16source & 0xFF);
    u8output[1] = ((u16source >> 8) & 0xFF);

    u8bytesHandled = sizeof(uint16_t);
  }
  else
  {
    u8output[0] = ((u16source >> 8) & 0xFF);
    u8output[1] = (u16source & 0xFF);

    u8bytesHandled = sizeof(uint16_t);
  }

  return u8bytesHandled;
}

uint8_t bit_insert_u8_in_u16(BitEdianess_t edianess, uint8_t *u8input, uint16_t *u16output)
{
  uint8_t u8bytesHandled = 0;

  if (edianess == LITTLE_ENDIAN)
  {
    *u16output = u8input[0];
    *u16output |= ((uint16_t)u8input[1] << 8);

    u8bytesHandled = sizeof(uint16_t);
  }
  else
  {
    *u16output = ((uint16_t)u8input[0] << 8);
    *u16output |= u8input[1];

    u8bytesHandled = sizeof(uint16_t);
  }

  return u8bytesHandled;
}

uint8_t bit_insert_u8_in_u32(BitEdianess_t edianess, uint8_t *u8input, uint32_t *u32output)
{
  uint8_t u8bytesHandled = 0;

  if (edianess == LITTLE_ENDIAN)
  {
    *u32output = u8input[0];
    *u32output |= ((uint32_t)u8input[1] << 8);
    *u32output |= ((uint32_t)u8input[2] << 16);
    *u32output |= ((uint32_t)u8input[3] << 24);

    u8bytesHandled = sizeof(uint32_t);
  }
  else
  {
    *u32output  = ((uint32_t)u8input[0] << 24);
    *u32output |= ((uint32_t)u8input[1] << 16);
    *u32output |= ((uint32_t)u8input[2] << 8);
    *u32output |= u8input[3];

    u8bytesHandled = sizeof(uint32_t);
  }

  return u8bytesHandled;
}


uint16_t bit_swap_endianess_u16(uint16_t u16ValueToSwap)
{
  return (u16ValueToSwap << 8) | (u16ValueToSwap >> 8 );
}

uint32_t bit_swap_endianess_u32(uint32_t u32ValueToSwap )
{
  u32ValueToSwap = ((u32ValueToSwap << 8) & 0xFF00FF00 ) | ((u32ValueToSwap >> 8) & 0xFF00FF );
  return (u32ValueToSwap << 16) | (u32ValueToSwap >> 16);
}


/* Private functions ---------------------------------------------------*/


/******************* (C) COPYRIGHT 2023*****END OF FILE****/
