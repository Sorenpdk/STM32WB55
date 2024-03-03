/**
  ******************************************************************************
  * @file    SP_DMA.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    03/03-2024 (DD/MM-YYYY)
  * @brief   Source for SP_DMA.h file.
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
#include "SP_DMA.h"

/* Defines ------------------------------------------------------------*/
#define USARTx               		LPUART1
#define USARTx_TX_DMA_STREAM 		DMA1_Channel1


/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/
/*
 * Channel configuration procedure
The following sequence is needed to configure a DMA channel x:
1. Set the peripheral register address in the DMA_CPARx register.
The data is moved from/to this address to/from the memory after the peripheral event,
or after the channel is enabled in memory-to-memory mode.
2. Set the memory address in the DMA_CMARx register.
The data is written to/read from the memory after the peripheral event or after the
channel is enabled in memory-to-memory mode.
3. Configure the total number of data to transfer in the DMA_CNDTRx register.
After each data transfer, this value is decremented.
4. Configure the parameters listed below in the DMA_CCRx register:
– the channel priority
– the data transfer direction
– the circular mode
– the peripheral and memory incremented mode
– the peripheral and memory data size
– the interrupt enable at half and/or full transfer and/or transfer error
5. Activate the channel by setting the EN bit in the DMA_CCRx register
 */
/*
 * Channel state and disabling a channel
A channel x in the active state is an enabled channel (read DMA_CCRx.EN = 1). An active
channel x is a channel that must have been enabled by the software (DMA_CCRx.EN set
to 1) and afterwards with no occurred transfer error (DMA_ISR.TEIFx = 0). In case there is a
transfer error, the channel is automatically disabled by hardware (DMA_CCRx.EN = 0).

The three following use cases may happen:
Suspend and resume a channel
This corresponds to the two following actions:
– An active channel is disabled by software (writing DMA_CCRx.EN = 0 whereas
DMA_CCRx.EN = 1).
– The software enables the channel again (DMA_CCRx.EN set to 1) without
reconfiguring the other channel registers (such as DMA_CNDTRx, DMA_CPARx
and DMA_CMARx).
This case is not supported by the DMA hardware, that does not guarantee that the
remaining data transfers are performed correctly.
Stop and abort a channel
If the application does not need any more the channel, this active channel can be
disabled by software. The channel is stopped and aborted but the DMA_CNDTRx
register content may not correctly reflect the remaining data transfers versus the
aborted source and destination buffer/register.
Abort and restart a channel
This corresponds to the software sequence: disable an active channel, then
reconfigure the channel and enable it again.
This is supported by the hardware if the following conditions are met:
– The application guarantees that, when the software is disabling the channel, a
DMA data transfer is not occurring at the same time over its master port. For
example, the application can first disable the peripheral in DMA mode, in order to
ensure that there is no pending hardware DMA request from this peripheral.
– The software must operate separated write accesses to the same DMA_CCRx
register: First disable the channel. Second reconfigure the channel for a next block
transfer including the DMA_CCRx if a configuration change is needed. There are
read-only DMA_CCRx register fields when DMA_CCRx.EN=1. Finally enable
again the channel.
When a channel transfer error occurs, the EN bit of the DMA_CCRx register is cleared by
hardware. This EN bit cannot be set again by software to reactivate the channel x, until the
TEIFx bit of the DMA_ISR register is set.
 */
void drv_DMA_init(void)
{
  // Enable DMA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN;

    // Configure DMA channel for USART Tx
    USARTx_TX_DMA_STREAM->CCR &= ~DMA_CCR_EN;       // Disable DMA channel
    while (USARTx_TX_DMA_STREAM->CCR & DMA_CCR_EN); // Wait until DMA is disabled

    // Configure DMA Mux for USART1 TX
     DMAMUX1_Channel0->CCR |= (0x0F << 0);

     USARTx_TX_DMA_STREAM->CCR |= DMA_CCR_DIR;            // Memory-to-peripheral mode

     USARTx_TX_DMA_STREAM->CCR |= DMA_CCR_MINC;             // Memory increment mode
     USARTx_TX_DMA_STREAM->CCR |= DMA_CCR_TCIE;             // Transfer complete interrupt enable

  /* In circular mode, after the last data transfer, the DMA_CNDTRx register is automatically
reloaded with the initially programmed value. The current internal address registers are
reloaded with the base address values from the DMA_CPARx and DMA_CMARx registers. */
}

void DMA_Configuration(uint8_t* data, uint16_t size)
{
    // Enable DMA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN;

    // Configure DMA channel for USART Tx
    USARTx_TX_DMA_STREAM->CCR &= ~DMA_CCR_EN;       // Disable DMA channel
    while (DMA1_Channel1->CCR & DMA_CCR_EN); // Wait until DMA is disabled

    USARTx_TX_DMA_STREAM->CPAR = (uint32_t)(&USARTx->TDR); // Set peripheral address
    USARTx_TX_DMA_STREAM->CMAR = (uint32_t)data;          // Set memory address
    USARTx_TX_DMA_STREAM->CNDTR = size;                    // Set data size
    DMAMUX1_Channel0->CCR |= (0x0F << 0);
    USARTx_TX_DMA_STREAM->CCR |= DMA_CCR_DIR | DMA_CCR_CIRC;            // Memory-to-peripheral mode

    USARTx_TX_DMA_STREAM->CCR |= DMA_CCR_MINC;             // Memory increment mode
    USARTx_TX_DMA_STREAM->CCR |= DMA_CCR_TCIE;             // Transfer complete interrupt enable

    // Configure NVIC for DMA
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  }

/* Private functions ---------------------------------------------------*/
void DMA_IRQHandler(void)
{
  if(DMA1->ISR & DMA_ISR_TCIF3)
    {
      // DMA transfer complete, perform any cleanup or additional actions here
      DMA1->IFCR |= DMA_IFCR_CTCIF3; // Clear transfer complete flag
  }
}



/******************* (C) COPYRIGHT 2023*****END OF FILE****/
