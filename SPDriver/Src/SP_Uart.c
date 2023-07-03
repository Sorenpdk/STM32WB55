/**
  ******************************************************************************
  * @file    SP_Uart.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    18/06-2023 (DD/MM-YYYY)
  * @brief   Source for SP_Uart.h file.
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
#include "SP_Uart.h"
#include "SP_GPIO.h"
#include "ringBuffer.h"
/* Defines ------------------------------------------------------------*/
#define TX_BUFFER_SIZE 16
#define RX_BUFFER_SIZE 16

/* Private function prototypes ----------------------------------------*/
static uint32_t calculateBaudRateValue(uint32_t baudrate);
static void set_UART_wordLength(UART_init_t* pUARTInit);
static void set_UART_baudRate(UART_init_t* pUARTInit);
static void set_UART_parity(UART_init_t* pUARTInit);
static void set_UART_stopBits(UART_init_t* pUARTInit);

/* Global variables ---------------------------------------------------*/
static sRingbuf_t uartTxBuffer;
static sRingbuf_t uartRxBuffer;
static uint8_t au8TxBuf[TX_BUFFER_SIZE];
static uint8_t au8RxBuf[RX_BUFFER_SIZE];

/* Public functions ----------------------------------------------------*/
void drv_uart_init(UART_init_t* pUARTInit)
{
  ringBuffer_init(&uartTxBuffer, au8TxBuf, TX_BUFFER_SIZE);
  ringBuffer_init(&uartRxBuffer, au8RxBuf, RX_BUFFER_SIZE);

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; /* CPU1 USART1 clocks enable */
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  /* https://www.st.com/resource/en/datasheet/stm32wb55cc.pdf  AF7  USART1 PB6 -> USART1_ TX PB7 -> USART1_ RX
  GPIOx_AFRL 0111: AF7*/
  GPIOB->AFR[0] |= (0x07 << GPIO_AFRL_AFSEL7_Pos);
  GPIOB->AFR[0] |= (0x07 << GPIO_AFRL_AFSEL6_Pos);

  GPIO_init_t USART_GPIO_PB5 = { .gpioType = GPIO_TYPE_PUSH_PULL,
			         .gpioMode = GPIO_MODER_ALTERNATE,
			         .gpioSpeed = GPIO_SPEED_HIGH,
			         .gpioPort = GPIO_PORTB,
				 .gpioPuPd = GPIO_NO_PULL,
				 .u8pinNumber = 7                 };

  GPIO_init_t USART_GPIO_PB6 = { .gpioType = GPIO_TYPE_PUSH_PULL,
  			         .gpioMode = GPIO_MODER_ALTERNATE,
  			         .gpioSpeed = GPIO_SPEED_HIGH,
  			         .gpioPort = GPIO_PORTB,
  				 .gpioPuPd = GPIO_NO_PULL,
  			         .u8pinNumber = 6	          };

  drv_GPIO_init(&USART_GPIO_PB5);
  drv_GPIO_init(&USART_GPIO_PB6);

  // Disable USART
   USART1->CR1 &= ~USART_CR1_UE;
   // Clear interrupt flags
    USART1->ICR = USART_ICR_TCCF;

   /* In asynchronous mode, the following bits must be kept cleared: LINEN and CLKEN bits in the USART_CR2 register */
  USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART1->CR3 &= ~((USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  set_UART_stopBits(pUARTInit);
  set_UART_wordLength(pUARTInit);
  set_UART_baudRate(pUARTInit);
  set_UART_parity(pUARTInit);

  /* Oversampling */
  USART1->CR1 &= ~USART_CR1_OVER8; /* 16 */

  /* CR2 Bit 19 MSBFIRST: Most significant bit first */
  // Enable USART1, transmitter, and receiver
  USART1->CR1 |= USART_CR1_TE | USART_CR1_UE | USART_CR1_RE  | USART_CR1_TCIE;

  // Clear the TXE interrupt enable bit
  USART1->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE); /* Clear TXEIE and TC interrupts */

  NVIC_SetPriority(USART1_IRQn, 0);			// Set Priority to 1
  NVIC_EnableIRQ(USART1_IRQn);				// Enable interrupt of USART1 peripheral
}


void drv_uart_trasnmit(uint8_t *pu8Data, uint16_t u16DataLengthBytes)
{
  for(uint16_t u16idx = 0; u16idx < u16DataLengthBytes; ++u16idx)
  {
    if((USART1->ISR & USART_ISR_TXE)) /* Is transmission buffer empty */
    {
      ringBuffer_put(&uartTxBuffer, pu8Data[u16idx]);
      USART1->CR1 |= USART_CR1_TXEIE; /* We have a byte to send, allow interrupt */
    }
    while(!(USART1->ISR & USART_ISR_TC)); /* Wait here while we service the interrupt */
  }
}


/* Private functions ---------------------------------------------------*/
static void set_UART_stopBits(UART_init_t* pUARTInit)
{
  switch (pUARTInit->uartStopBits)
  {
    case UART_STOP_BITS_1:
      USART1->CR2 &= ~(0x01UL << USART_CR2_STOP_Pos);
      USART1->CR2 &= ~(0x01UL << (USART_CR2_STOP_Pos + 1U));
      break;

    case UART_STOP_BITS_2:
      USART1->CR2 |= (0x2UL << USART_CR2_STOP_Pos) ;
      break;

    default: break;
  }
}

static void set_UART_wordLength(UART_init_t* pUARTInit)
{
  switch (pUARTInit->uartWordLength)
   {
     case UART_WORD_LEGTH_8_BITS:
     {
       USART1->CR1 &= ~USART_CR1_M1;
       USART1->CR1 &= ~USART_CR1_M0;
     } break;

     case UART_WORD_LEGTH_7_BITS: break;
     case UART_WORD_LEGTH_9_BITS: break;
     default: break;
   }
}

static void set_UART_baudRate(UART_init_t* pUARTInit)
{
  uint32_t baudValue = 0;
  switch (pUARTInit->uartBaud)
  {
    case UART_BAUD_9600:  { baudValue = calculateBaudRateValue(9600);  } break;
    case UART_BAUD_38400: { baudValue = calculateBaudRateValue(38400); } break;
    case UART_BAUD_115200: { baudValue = calculateBaudRateValue(115200);} break;
    default: break;
  }
   USART1->BRR = baudValue;
}

static void set_UART_parity(UART_init_t* pUARTInit)
{
  switch (pUARTInit->uartParity)
  {
    case UART_PARITY_NONE: { USART1->CR1 &= ~USART_CR1_PCE; } break;
    case UART_PARITY_EVEN:
    {
      USART1->CR1 |= USART_CR1_PCE;
      USART1->CR1 &= ~USART_CR1_PS;
    } break;

    case UART_PARITY_ODD: { USART1->CR1 |= USART_CR1_PCE | USART_CR1_PS; } break;
    default: break;
  }
}


/* TODO: Fix proper clock calculations from prescalres etc, for CPU1 -> HCLK1 -> APB prescalers etc. */
/* BRR[3:0]
When OVER8 = 0, BRR[3:0] = USARTDIV[3:0].*/
/* Bits 15:0 BRR[15:0]: USART baud rate
BRR[15:4]
BRR[15:4] = USARTDIV[15:4]
se 1055 in ref manual
 *  */
/* The BRR uses a fixed-point representation, ie Q12.4

int((104.1875 * 16) + 0.5) = 1667 = 0x683

Personally I've been using BRR = APBCLK / BAUD for 13+ years, it is simpler to explain/compute. */
static uint32_t calculateBaudRateValue(uint32_t u32baudrate)
{
  uint32_t APBCLK = 64000000;
  uint32_t u32res = 0;

  u32res = (APBCLK / u32baudrate );

  return u32res;
}

/* The TE bit must be set before writing the data to be transmitted to the USART_TDR.
The TE bit should not be reset during data transmission. Resetting the TE bit during the
transmission corrupts the data on the TX pin as the baud rate counters get frozen. The
current data being transmitted are then lost.
An idle frame is sent when the TE bit is enabled. */

void drv_uart_IRQHandler(void)
{
  if (USART1->ISR & USART_ISR_TC) /* If a transfer is complete, byte by byte */
  {
    if (uartTxBuffer.u16elementsInBuffer > 0) /* Do we have anything to send */
    {
      USART1->TDR = ringBuffer_get(&uartTxBuffer);
      while(!(USART1->ISR & USART_ISR_TC));  /* Wait here until the transfer is complete */
    }

    USART1->CR1 &= ~(USART_CR1_TXEIE); /* Disable the transmission buffer empty interrupt, we will enable this once we have something to send */
  }
}

/******************* (C) COPYRIGHT 2023*****END OF FILE****/
