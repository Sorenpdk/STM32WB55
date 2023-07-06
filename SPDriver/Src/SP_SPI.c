/**
  ******************************************************************************
  * @file    SP_SPI.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    26/06-2023 (DD/MM-YYYY)
  * @brief   Source for SP_SPI.h file.
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
#include "SP_SPI.h"
#include "SP_GPIO.h"

#include "stm32wbxx_hal.h"
/* Defines ------------------------------------------------------------*/
#define SPI_CS_PIN   4
#define TX_BUFFER_SIZE 56
#define RX_BUFFER_SIZE 56

/* Private function prototypes ----------------------------------------*/
static void drv_SPI_assertCS(bool_t bValue);
static void drv_SPI_transmit(uint8_t u8data);
/* Global variables ---------------------------------------------------*/
static sRingbuf_t spiTxBuffer;
static sRingbuf_t spiRxBuffer;
static uint8_t au8TxBuf[TX_BUFFER_SIZE];
static uint8_t au8RxBuf[RX_BUFFER_SIZE];

/* Public functions ----------------------------------------------------*/

void drv_SPI_init(void)
{
  /*
  PA6 SS

  PA11 SPI1_ MISO
  PA12 SPI1_ MOSI
  PA5 SPI1_ SCK*/
  ringBuffer_init(&spiTxBuffer, au8TxBuf, TX_BUFFER_SIZE);
  ringBuffer_init(&spiRxBuffer, au8RxBuf, RX_BUFFER_SIZE);

  GPIO_init_t SPI_GPIO_CS_PA4 = { .gpioType = GPIO_TYPE_PUSH_PULL,
				   .gpioMode = GPIO_MODER_OUTPUT,
				   .gpioSpeed = GPIO_SPEED_HIGH,
				   .gpioPort = GPIO_PORTA,
				   .gpioPuPd = GPIO_PULLUP,
				   .u8pinNumber = SPI_CS_PIN                 };

  GPIO_init_t SPI_GPIO_MISO_PA6 = { .gpioType = GPIO_TYPE_PUSH_PULL,
				     .gpioMode = GPIO_MODER_ALTERNATE,
				     .gpioSpeed = GPIO_SPEED_HIGH,
				     .gpioPort = GPIO_PORTA,
				     .gpioPuPd = GPIO_PULLUP,
				     .u8pinNumber = 6                 };

  GPIO_init_t SPI_GPIO_MOSI_PA7 = { .gpioType = GPIO_TYPE_PUSH_PULL,
  				     .gpioMode = GPIO_MODER_ALTERNATE,
  				     .gpioSpeed = GPIO_SPEED_HIGH,
  				     .gpioPort = GPIO_PORTA,
  				     .gpioPuPd = GPIO_PULLUP,
  				     .u8pinNumber = 7                 };

  GPIO_init_t SPI_GPIO_SCK_PA5 = { .gpioType = GPIO_TYPE_PUSH_PULL,
				   .gpioMode = GPIO_MODER_ALTERNATE,
				   .gpioSpeed = GPIO_SPEED_HIGH,
				   .gpioPort = GPIO_PORTA,
				   .gpioPuPd = GPIO_PULLUP,
				   .u8pinNumber = 5                 };

  drv_GPIO_init(&SPI_GPIO_CS_PA4);
  drv_GPIO_init(&SPI_GPIO_MISO_PA6);
  drv_GPIO_init(&SPI_GPIO_MOSI_PA7);
  drv_GPIO_init(&SPI_GPIO_SCK_PA5);

  GPIOA->AFR[0] |= (0x05 << GPIO_AFRL_AFSEL5_Pos);
  GPIOA->AFR[0] |= (0x05 << GPIO_AFRL_AFSEL6_Pos);
  GPIOA->AFR[0] |= (0x05 << GPIO_AFRL_AFSEL7_Pos);
/*
  AF5
  PA6 SPI1_ MISO
  PA7 SPI1_ MOSI
  PA5 SPI1_ SCK
*/

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  SPI1->CR1 |= SPI_CR1_SSM; /* Software slave select */

  SPI1->CR1 |= SPI_CR1_BR_0; /* 001: fPCLK DIV 4 */

  SPI1->CR1 |= SPI_CR1_MSTR; /* Master */


  SPI1->CR1 |= SPI_CR1_CPOL; /* CK to 1 when idle */
  SPI1->CR1 |= SPI_CR1_CPHA; /* Clock phase 1: The second clock transition is the first data capture edge */

  SPI1->CR1 |= SPI_CR1_BR_2;
  SPI1->CR1 |= SPI_CR1_BR_1;
  SPI1->CR1 |= SPI_CR1_BR_0;

  // this fucks up SPI1->CR1 |= SPI_CR1_LSBFIRST;
 /* 000: fPCLK/2 */
  /* 100: fPCLK/32 */
 // SPI1->CR2 |= SPI_CR2_FRXTH;   /* 1/4 (8 bit) */

  SPI1->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 ); // | SPI_CR2_DS_3);
  SPI1->CR2 &= ~(SPI_CR2_DS_3); /* 0111: 8-bit */
 /* If software attempts to write one of the “Not used” values, they are forced to the value “0111”
(8-bit) */

  ///* Tx buffer empty interrupt enable */

  SPI1->CR2 |= SPI_CR2_SSOE; /* 1: SS output is enabled in master mode and when the SPI interface is enabled. The SPI
interface cannot work in a multimaster environment. */

  SPI1->CR1 |= SPI_CR1_SPE; /* SPI enable */
 SPI1->CR2 |= SPI_CR2_RXNEIE;
 SPI1->CR2 |= SPI_CR2_TXEIE;
  //NVIC_SetPriority(SPI1_IRQn, 0);			// Set Priority to 1
  //NVIC_EnableIRQ(SPI1_IRQn);				// Enable interrupt of USART1 peripheral
}


void drv_SPI_idle(void)
{

}

uint8_t drv_SPI_transmit_singleByte(uint8_t u8Data)
{
  drv_SPI_assertCS(false);

  drv_SPI_transmit(u8Data);


  uint8_t receivedData = (uint8_t)((SPI1->DR & 0xFF00) >> 8);

  drv_SPI_assertCS(true);

  return receivedData;
}

void drv_SPI_transmit_nBytes(uint8_t* pu8Data, uint16_t u16dataLength)
{
  drv_SPI_assertCS(false);

  for(uint16_t u16idx = 0; u16idx < u16dataLength; ++u16idx)
  {
    drv_SPI_transmit(pu8Data[u16idx]);
  }

  drv_SPI_assertCS(true);
}


static void drv_SPI_transmit(uint8_t u8data)
{

  while (!(SPI1->SR & SPI_SR_TXE)) {} // Wait for the transmit buffer to be empty
  SPI1->DR = u8data;

    // Enable transmit buffer empty interrupt
  // SPI1->CR2 |= SPI_CR2_TXEIE;

  while (SPI1->SR & SPI_SR_BSY){} // Wait for the SPI peripheral to finish the transmission

}


static void drv_SPI_assertCS(bool_t bValue)
{
  drv_GPIO_set_pin(GPIO_PORTA, bValue, SPI_CS_PIN);
}

/*
void drv_SPI_IRQHandler(void)
{
  if(SPI1->SR & SPI_SR_TXE)
  {
    SPI1->CR2 &= ~(SPI_CR2_TXEIE); // Disable transmit buffer empty interrupt
  }

  if (SPI1->SR & SPI_SR_RXNE)
  {
    uint8_t receivedData = (uint8_t)((SPI1->DR & 0xFF00) >> 8);

    if(receivedData != 0xFF)
    {
	ringBuffer_put(&spiRxBuffer, receivedData);
    }
  }
  if(SPI1->SR & SPI_SR_OVR)
  {
    SPI1->SR &= ~SPI_SR_OVR;
  }
}
*/
sRingbuf_t* drv_SPI_getRxDataPtr(void)
{
  return &spiRxBuffer;
}
/*
 *
    fn read_reg(&mut self, register: Register) -> Result<u8, Error<SpiError, PinError>> {
        self.chip_select()?;
        let request = 0b1000_0000 | register.addr(); // set the read bit
        let result = self.write_then_read(request);
        self.chip_deselect()?;
        result
    }
 */


/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2023*****END OF FILE****/

