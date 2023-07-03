/**
  ******************************************************************************
  * @file    simpleFSM.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    05/11-2022 (DD/MM-YYYY)
  * @brief   Source for simpleFSM.h file.
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
#include "simpleFSM.h"

/* Defines ------------------------------------------------------------*/


/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/
static const state_table_t* states;

/* Public functions ----------------------------------------------------*/
void simple_fsm_init(fsm_t* pfsm, uint8_t u8sizeBytes, const state_table_t* pstates)
{
  pfsm->u8statesCount = u8sizeBytes;
  states = pstates;
}

void simple_fsm_idle(void)
{

}

void simple_fsm_run(fsm_t* pfsm)
{
  pfsm->u8currentState = pfsm->u8nextState;

  if(states[pfsm->u8currentState].entryFunction != NULL)
  {
      states[pfsm->u8currentState].entryFunction();
  }

  states[pfsm->u8currentState].actionFunction((int*)&pfsm->u8nextState);

  if(states[pfsm->u8currentState].exitFunction != NULL)
  {
     states[pfsm->u8currentState].exitFunction();
  }

  pfsm->u8currentState = pfsm->u8nextState;
}

/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2022*****END OF FILE****/
