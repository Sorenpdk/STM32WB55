/**
  ******************************************************************************
  * @file    accelerometer_lis2dw12.c
  * @author  Søren Pørksen
  * @version V1.0.0
  * @date    21/06-2023 (DD/MM-YYYY)
  * @brief   Source for accelerometer_lis2dw12.h file.
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
#include "accelerometer_lis2dw12.h"
#include "SP_SPI.h"
#include "ringBuffer.h"
#include "string.h"
/* Defines ------------------------------------------------------------*/
#define READ_CMD_BIT_MASK		0x80
#define OUT_TEMPERATURE_L_RO  		0x0D /* Temperature output register in 12-bit resolution */
#define OUT_TEMPERATURE_H_RO  		0x0E /* Temperature output register in 12-bit resolution */
#define WHO_AM_I_RO  			0x0F   /* Will respond with 0x44 */
#define CTRL1_RW 			0x20
#define CTRL2_RW 			0x21
#define CTRL3_RW 			0x22
#define CTRL4_INT1_PAD_CTRL_RW  	0x23
#define CTRL5_INT2_PAD_CTRL_RW  	0x24
#define CTRL6_RW 			0x25
#define OUT_T_RO 			0x26
#define STATUS_RO 			0x27
#define OUT_X_L_RO 			0x28

#define OUT_X_H_RO 			0x29
#define OUT_Y_L_RO 			0x2A
#define OUT_Y_H_RO 			0x2B
#define OUT_Z_L_RO 			0x2C
#define OUT_Z_H_RO 			0x2D

#define STATUS_DUP_RO 			0x37
#define FIFO_CTRL			0x2E

#define TAP_SRC_RO			0x39


float convertTemperature(uint16_t twosComplementValue);
static int8_t convertTwosCompToInt(uint8_t u8twoscomp);

/*
FIFO_CTRL R/W 2E 00101110 00000000 FIFO control register
FIFO_SAMPLES R 2F 00101111 00000000 Unread samples stored in FIFO
TAP_THS_X R/W 30 00110000 00000000
TAP_THS_Y R/W 31 00110001 00000000 Tap thresholds
TAP_THS_Z R/W 32 00110010 00000000
INT_DUR R/W 33 00110011 00000000 Interrupt duration
WAKE_UP_THS R/W 34 00110100 00000000
Tap/double-tap selection,
inactivity enable,
wakeup threshold
WAKE_UP_DUR R/W 35 00110101 00000000 Wakeup duration
FREE_FALL R/W 36 00110110 00000000 Free-fall configuration
STATUS_DUP R 37 00110111 00000000 Status register
WAKE_UP_SRC R 38 00111000 00000000 Wakeup source
TAP_SRC R 39 00111001 00000000 Tap source
SIXD_SRC R 3A 00111010 00000000 6D source

ALL_INT_SRC R 3B 00111011 00000000
X_OFS_USR R/W 3C 00111100 00000000
Y_OFS_USR R/W 3D 00111110 00000000
Z_OFS_USR R/W 3E 00000100 00000000
CTRL7 R/W 3F 00000100 00000000


R = read-only register, R/W = readable/writable register
Registers marked as Reserved must not be changed. Writing to those registers may cause permanent damage to
the device.
The content of the registers that are loaded at boot should not be changed. They contain the factory calibration
values. Their content is automatically restored when the device is powered up. */
static void configure(void);

void acc_init(void)
{
  configure();
}

static uint8_t u8whoami;
void acc_getWhoAmI(void)
{
  uint8_t u8cmd[1] = {(READ_CMD_BIT_MASK | WHO_AM_I_RO)};
  drv_SPI_assertCS(false);

  drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));
  drv_SPI_receive_nBytes(&u8whoami, 1);


  drv_SPI_assertCS(true);
}


static void configure(void)
{
  uint8_t u8cmd[2] = {CTRL1_RW, 0x9A};

  drv_SPI_assertCS(0);

  drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));

  drv_SPI_assertCS(1);




   u8cmd[0] = CTRL2_RW;
   u8cmd[1] = 0x16;

   drv_SPI_assertCS(0);

   drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));

   drv_SPI_assertCS(1);



   u8cmd[0] = 0x2E;
   u8cmd[1] = 0;

   drv_SPI_assertCS(0);

   drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));

   drv_SPI_assertCS(1);


   u8cmd[0] = CTRL3_RW;
     u8cmd[1] = 0x03;

     drv_SPI_assertCS(0);

     drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));

     drv_SPI_assertCS(1);
}





#define WAKE_UP_THS 0x34

/* Private function prototypes ----------------------------------------*/
static uint8_t u8tempHigh;
static uint8_t u8tempLow;
uint16_t acc_getTemperature(void)
{
  uint8_t u8cmd[2] = {0};
  u8cmd[0] = CTRL3_RW;
   u8cmd[1] = 0x03;

   drv_SPI_assertCS(0);

   drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));

   drv_SPI_assertCS(1);



   u8cmd[0] = (READ_CMD_BIT_MASK | OUT_X_L_RO);
   drv_SPI_assertCS(0);

   drv_SPI_transmit_nBytes(u8cmd, 1);
   drv_SPI_receive_nBytes(&u8tempHigh, 1);

   drv_SPI_assertCS(1);

      u8cmd[0] = (READ_CMD_BIT_MASK | OUT_X_H_RO);

      drv_SPI_assertCS(0);

      drv_SPI_transmit_nBytes(u8cmd, 1);
      drv_SPI_receive_nBytes(&u8tempHigh, 1);

      drv_SPI_assertCS(1);

    return 1;
}




uint8_t x = 0x00;

static int8_t convertTwosCompToInt(uint8_t u8twoscomp)
{
  int8_t y = (u8twoscomp ^ 0xFF) + 1; // Invert bits and add 1
  if (u8twoscomp & (1 << 7)) // Check if 8th bit is set (meaning signed)
  {
    return (~(y) + 1); // invert back from unsigned to signed.
  }

  return u8twoscomp; // if no sign bit then fine...
}

/*
There is no "128" in a signed byte. The range is

0 to 127 : 128 values
-1 to -128 : 128 value
*/









float convertTemperature(uint16_t temperatureOutput)
{
    uint8_t lsb = (temperatureOutput & 0xFF);  // Extract the least significant byte
    uint8_t msb = (temperatureOutput >> 8) & 0xFF;  // Extract the most significant byte

    float baselineTemperature = 25.0f;  // Baseline temperature for 0 LSB
    float sensitivity = 16.0f;  // Sensitivity: 16 LSB/°C

    // Combine the bytes into the signed value
    int16_t combinedValue = (int16_t)((msb << 8) | lsb);

    // Calculate the temperature
    float temperature = baselineTemperature + (combinedValue / sensitivity);

    return temperature;
}

/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/

/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2023*****END OF FILE****/
