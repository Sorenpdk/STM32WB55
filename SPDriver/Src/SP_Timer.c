/**
  ******************************************************************************
  * @file    SP_Timer.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    27/02-2024 (DD/MM-YYYY)
  * @brief   Source for SP_Timer.h file.
   TODO: Fill this module handles.....
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

/* Includes -----------------------------------------------------------*/
#include "SP_Timer.h"

/* Defines ------------------------------------------------------------*/
/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/
static volatile bool_t bFlagStatus_UpdateInterrupt;
static volatile bool_t bFlagStatus_CC1;
static volatile bool_t bFlagStatus_CC2;
/* Public functions ----------------------------------------------------*/

/*
 * UpdateEvent = (timerperiph_clk_hz)/(prescalerval + 1)(periode(ARR) +1)
 *
 *
 */



void drv_timer_init(void)
{

    // Enable the peripheral clock for Timer 2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;


    TIM2->PSC = 64000; // Set timer 3 prescaler
    TIM2->ARR = 0xFFFF; //Set timer 3 auto reload value
    TIM2->CR1 &= ~(3 << TIM_CR1_CMS_Pos); //selecting edge aligned PWM
    TIM2->CR1 |= TIM_CR1_ARPE; //Enable auto-reload preload

    TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);//Capture compare 1 disable
    TIM2->CCER &= ~TIM_CCER_CC1P;//Capture compare polarity active high
    TIM2->CCMR1 &=  ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);//CC1 channel is output
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);//OC no output selected
    TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);//Capture compare 1 enable
    TIM2->EGR |= TIM_EGR_UG;//Generate update
    TIM2->CR1 |= TIM_CR1_CEN; //Enable the counter
    TIM2->SR = 0x00;

    TIM2->CCR1 = 5000;
    TIM2->CCR2 = 50000;
    TIM2->CNT = 0;
    TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE);//Enable CH1 output compare interrupt bit

    NVIC_SetPriority(TIM2_IRQn, 0); //Set timer 3 ISR priority
    NVIC_EnableIRQ(TIM2_IRQn); //Enable timer 3 ISR
}




/*
 * or to check
 against the UIF flag status¹⁵
 */
static volatile uint32_t u32CCR1_CNT;
static volatile uint32_t u32CCR2_CNT;
void drv_timer_IRQHandler(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
     if((TIM2->SR & TIM_SR_CC1IF) && (TIM2->DIER & TIM_DIER_CC1IE))
     {

	 TIM2->SR &= ~TIM_SR_CC1IF;

	 u32CCR1_CNT++;

     }

     if((TIM2->SR & TIM_SR_CC2IF) && (TIM2->DIER & TIM_DIER_CC2IE))
      {


       TIM2->SR &= ~TIM_SR_CC2IF;
       u32CCR2_CNT++;


      }


  }
}

/* Private functions ---------------------------------------------------*/


/******************* (C) COPYRIGHT 2024*****END OF FILE****/
