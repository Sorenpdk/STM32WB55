/**
  ******************************************************************************
  * @file    simpleFSM.h
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    08/10-2022 (DD/MM-YYYY)
  * @brief   Header for simpleFSM.c file.
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
#ifndef __SIMPLE_FSM_H
#define __SIMPLE_FSM_H

/* Includes ------------------------------------------------------------------*/
#include "common.h"

/* Exported types ------------------------------------------------------------*/
typedef void(*pfnEntry)(void);
typedef void(*pfnAction)(int* pi16NextState);
typedef void(*pfnExit)(void);

typedef struct
{
  uint8_t State;
  pfnEntry entryFunction;
  pfnAction actionFunction;
  pfnExit exitFunction;
} state_table_t;

typedef struct
{
   uint8_t u8nextState;
   uint8_t u8currentState;
   state_table_t* statesTable;
   uint8_t u8statesCount;
} fsm_t;
/* Exported constants --------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void simple_fsm_init(fsm_t* pfsm, uint8_t u8sizeBytes, const state_table_t* pstates);
void simple_fsm_idle(void);
void simple_fsm_run(fsm_t* pfsm);

#endif /* __SIMPLE_FSM_H */

/******************* (C) COPYRIGHT *****END OF FILE****/



