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
#include "string.h"
#include "stm32wbxx_hal.h"
/* Defines ------------------------------------------------------------*/
#define SPI_CS_PIN   4

/* Private function prototypes ----------------------------------------*/


/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/

void drv_SPI_init(void)
{
  /*
  PA6 SS

  PA11 SPI1_ MISO
  PA12 SPI1_ MOSI
  PA5 SPI1_ SCK*/
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

  SPI1->CR1 |= SPI_CR1_MSTR; /* Master */

  SPI1->CR1 |= SPI_CR1_CPOL; /* CK to 1 when idle */
  SPI1->CR1 |= SPI_CR1_CPHA; /* Clock phase 1: The second clock transition is the first data capture edge */

  // Clear the existing BR bits
  SPI1->CR1 &= ~SPI_CR1_BR;

  // Set the desired baud rate divisor (e.g., divide by 64)
  SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;

 /* 000: fPCLK/2 */
  /* 100: fPCLK/32 */
  /* This bit is used to set the threshold of the RXFIFO that triggers an RXNE event */
  //SPI1->CR2 |= SPI_CR2_FRXTH;   /* 1/4 (8 bit) */

  //SPI1->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 ); // | SPI_CR2_DS_3);
  //SPI1->CR2 &= ~(SPI_CR2_DS_3); /* 0111: 8-bit */
 /* If software attempts to write one of the “Not used” values, they are forced to the value “0111”
(8-bit) */


  SPI1->CR2 |= SPI_CR2_SSOE; /* 1: SS output is enabled in master mode and when the SPI interface is enabled. The SPI
interface cannot work in a multimaster environment. */

  SPI1->CR1 |= SPI_CR1_SPE; /* SPI enable */
 SPI1->CR2 |= SPI_CR2_RXNEIE;
 SPI1->CR2 |= SPI_CR2_TXEIE;
}



void drv_SPI_transmitReceive(uint8_t* pu8TxData, uint8_t* pu8RxDataRx, uint16_t u16TxdataLength, uint16_t u16RxdataLength)
{
  uint16_t u16currentTxLength = 0;
  uint16_t u16currentRxLength = 0;

  /* Clear RXNE flag */
  (void)SPI1->DR;

  while (u16currentTxLength < u16TxdataLength)
  {
    while (!(SPI1->SR & SPI_SR_TXE)); // Wait for the transmit buffer to be empty
    *((__IO uint8_t *)&SPI1->DR) = pu8TxData[u16currentTxLength];

   while (!(SPI1->SR & SPI_SR_TXE)){}; // Wait for the transmit buffer to be empty

    *((__IO uint8_t *)&SPI1->DR) = 0x00; // Dummy write after the first command

   u16currentTxLength++;

    /* Wait until RXNE flag is set */
   if(u16RxdataLength > 0)
   {
    while (!(SPI1->SR & SPI_SR_RXNE)){};
    pu8RxDataRx[u16currentRxLength] = (uint8_t)(SPI1->DR >> 8) ;
    u16currentRxLength++;
   }
  }

  // Add a delay here to ensure previous data has been fully received
  for (volatile int i = 0; i < 10; i++);

  while (u16currentRxLength < u16RxdataLength)
  {
    *((__IO uint8_t *)&SPI1->DR) = 0x00;
    while (!(SPI1->SR & SPI_SR_RXNE));
    pu8RxDataRx[u16currentRxLength] = *((__IO uint8_t *)&SPI1->DR);

    u16currentRxLength++;
  }

  /* Wait until SPI is not busy */
  while ((SPI1->SR & SPI_SR_BSY) != 0) {};

}

void drv_SPI_idle(void)
{

}


void drv_SPI_transmit_nBytes(uint8_t* pu8Data, uint16_t u16dataLength)
{
  for(uint16_t u16idx = 0; u16idx < u16dataLength; ++u16idx)
  {
    while (!(SPI1->SR & SPI_SR_TXE)) {}  //Wait for the transmit buffer to be empty
    *((__IO uint8_t *)&SPI1->DR) = pu8Data[u16idx];
  }

  while (!(SPI1->SR & SPI_SR_TXE)) {} //Wait for the transmit buffer to be empty
  while (SPI1->SR & SPI_SR_BSY){}  //Wait for the SPI peripheral to finish the transmission

  if(SPI1->SR & SPI_SR_OVR)
  {
    uint8_t u8void = SPI1->DR;
    (void)u8void;
    u8void = SPI1->SR;
  }
}




void drv_SPI_receive_nBytes(uint8_t* pu8Data, uint16_t u16dataLength)
{
   for(uint16_t u16idx = 0; u16idx < u16dataLength; ++u16idx)
   {
      while (SPI1->SR & SPI_SR_BSY){}
      *((__IO uint8_t *)&SPI1->DR) = 0x00;
      while (!(SPI1->SR & SPI_SR_RXNE)) {}
      if(!(SPI1->SR & SPI_SR_OVR))
      {
	pu8Data[u16idx] = *((__IO uint8_t *)&SPI1->DR);
      }
   }

}


void drv_SPI_assertCS(bool_t bValue)
{
  while (SPI1->SR & SPI_SR_BSY){ }; // Wait for the SPI peripheral to finish the transmission
  drv_GPIO_set_pin(GPIO_PORTA, bValue, SPI_CS_PIN);
}

/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2023*****END OF FILE****/

