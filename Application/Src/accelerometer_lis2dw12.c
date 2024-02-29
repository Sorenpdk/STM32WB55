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
#define OUT_TEMPERATURE_L_RO  		0x0D
#define OUT_TEMPERATURE_H_RO  		0x0E
#define WHO_AM_I_RO  			0x0F  /* Will respond with 0x44 */
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




/* ODR [7:4] = 1001 High-Performance / Low-Power mode 1600/200 Hz
 * MODE[3:2] = 00 Low-Power Mode (12/14-bit resolution)
 * MODE[1:0] = 00 Low-Power Mode 1 (12-bit resolution)
 */
#define LIS2DW12_CTRL1_ADDRESS 		0x20

#define LIS2DW12_CTRL1_VALUE           ( ( 0x09  << 4)   \
				       | ( 0x00  << 2)   \
				       | ( 0x00  << 0)   )

/* BOOT      	[7]: 1: enabled - enables retrieving the correct trimming parameters
*  SOFT RESET	[6]: 0: disabled;
*  		[5]: bit must be set to 0
*  CS_PU_DISC   [4]: 0: pull-up connected to CS pin;
*  BDU 		[3]: 0: continuous update;
*  IF_ADD_INC   [2]: 0: disabled -Increment register address automatically.
*  I2C_DISABLE  [1]: 1: I²C mode disabled
*  SIM		[0]: SPI serial interface mode selection. Default value: 0 = 4-wire interface; */

#define LIS2DW12_CTRL2_ADDRESS 		0x21

#define LIS2DW12_CTRL2_VALUE           ( ( 0x01  << 7)   \
				       | ( 0x00  << 6)   \
				       | ( 0x00  << 5)   \
				       | ( 0x00  << 4)   \
				       | ( 0x00  << 3)   \
				       | ( 0x00  << 2)   \
				       | ( 0x01  << 1)   \
				       | ( 0x00  << 0)   )


/* ST	        [7:6]:  0 0 (00: Self-test disabled;
*  PP_OD	[5]: (0: push-pull;
*  LIR    	[4]: (0: interrupt request not latched;
*  H_LACTIVE	[3]: Interrupt active high, low (0: active high;
*  0   		[2]: 0: disabled -Increment register address automatically.
*  SLP_MODE_SEL [1]: 0: enabled with external trigger on INT2;
*  SLP_MODE_1	[0]: this bit is set to '1' logic, single data conversion on demand mode starts*/

#define LIS2DW12_CTRL3_ADDRESS 		0x22

#define LIS2DW12_CTRL3_VALUE           ( ( 0x00  << 7)   \
				       | ( 0x00  << 6)   \
				       | ( 0x00  << 5)   \
				       | ( 0x00  << 4)   \
				       | ( 0x00  << 3)   \
				       | ( 0x00  << 2)   \
				       | ( 0x00  << 1)   \
				       | ( 0x01  << 0)   )


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
static void lis2dw12_configure_ctrl_1(void);
static void lis2dw12_configure_ctrl_2(void);
static uint8_t get_temperature_MSB(void);
static uint8_t get_temperature_LSB(void);
static uint8_t get_z_axis_MSB(void);
static uint8_t get_z_axis_LSB(void);
static int16_t convertTwosCompToInt(uint16_t u16twoscomp);
void acc_init(void)
{
  configure();
  lis2dw12_configure_ctrl_1();

  uint8_t u8cmd3[2] = {0};
  u8cmd3[0] = CTRL3_RW;
  u8cmd3[1] = 0x03;

  drv_SPI_assertCS(0);

  drv_SPI_transmit_nBytes(u8cmd3,2);
  drv_SPI_assertCS(1);
}



static void lis2dw12_configure_ctrl_1(void)
{
  uint8_t u8registerValue = (CTRL1_ODR_1600_200_HZ | CTRL1_MODE_SINGLE_CONVERSION | CTRL1_LP_MODE_1);
  uint8_t u8cmd[2] = {CTRL1_RW, u8registerValue};
  drv_SPI_assertCS(FALSE);
  drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));
  drv_SPI_assertCS(TRUE);
}



static void lis2dw12_configure_ctrl_2(void)
{
  uint8_t u8registerValue = (CTRL1_ODR_1600_200_HZ | CTRL1_MODE_SINGLE_CONVERSION | CTRL1_LP_MODE_1);
  uint8_t u8cmd[2] = {CTRL2_RW, u8registerValue};
  drv_SPI_assertCS(FALSE);
  drv_SPI_transmit_nBytes(u8cmd, sizeof(u8cmd));
  drv_SPI_assertCS(TRUE);
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

#define LSB_PER_DEG_C   			16
#define LIS2DW12_TEMPERATURE_OFFSET_DEG_C  	25



float lis2dw12_get_z_sample_mg(void)
{
  float f32converted_Mg = 0.0f;

  int16_t i16RawFromTwosComplement = convertTwosCompToInt(lis2dw12_get_z_sample_raw());

  i16RawFromTwosComplement >>= 2;

  f32converted_Mg = (float)i16RawFromTwosComplement * 0.244f;

  return f32converted_Mg;
}


uint16_t lis2dw12_get_z_sample_raw(void)
{
  uint16_t u16ZAxisCombined = (get_z_axis_MSB() << 8) | get_z_axis_LSB();

  return u16ZAxisCombined;
}


static uint8_t get_z_axis_LSB(void)
{
   uint8_t u8ZAxisLSBRaw = 0;
   uint8_t au8cmd[1];
   au8cmd[0] = (READ_CMD_BIT_MASK | OUT_Z_L_RO);

   drv_SPI_assertCS(FALSE);
   drv_SPI_transmitReceive(au8cmd, &u8ZAxisLSBRaw, sizeof(au8cmd), sizeof(u8ZAxisLSBRaw));
   drv_SPI_assertCS(TRUE);

   return u8ZAxisLSBRaw;
}

static uint8_t get_z_axis_MSB(void)
{
   uint8_t u8ZAxisMSBRaw = 0;
   uint8_t au8cmd[1];
   au8cmd[0] = (READ_CMD_BIT_MASK | OUT_Z_H_RO);

   drv_SPI_assertCS(FALSE);
   drv_SPI_transmitReceive(au8cmd, &u8ZAxisMSBRaw, sizeof(au8cmd), sizeof(u8ZAxisMSBRaw));
   drv_SPI_assertCS(TRUE);

   return u8ZAxisMSBRaw;
}



void lis2dw12_generate_single_dataConversion(void)
{
  uint8_t au8cmd[2];
  au8cmd[0] = CTRL3_RW;
  au8cmd[1] = 0x01;

  drv_SPI_assertCS(FALSE);
  drv_SPI_transmit_nBytes(au8cmd, sizeof(au8cmd));
  drv_SPI_assertCS(TRUE);
}


uint8_t lis2dw12_get_whoAmI(void)
{
   uint8_t u8WhoAmI = 0;
   uint8_t au8cmd[1];
   au8cmd[0] = (READ_CMD_BIT_MASK | WHO_AM_I_RO);

   drv_SPI_assertCS(FALSE);
   drv_SPI_transmitReceive(au8cmd, &u8WhoAmI, sizeof(au8cmd), sizeof(u8WhoAmI));
   drv_SPI_assertCS(TRUE);

   return u8WhoAmI;
}


/* >>= 4; because the data sheet says, which is fucked up and isnt explained.
 * something something left justified etc.
 * 			TEMP MSB						    TEMP LSB
 * |TEMP11|TEMP10|TEMP09|TEMP08|TEMP07|TEMP06|TEMP05|TEMP04|		|TEMP3|TEMP2|TEMP1|TEMP0|0|0|0|0
 *
 * So with out the shift 1 LSB is 1 DEG because TEMP 0 is at bit pos 4, so we need to shift it down so 16LSB because 1 deg.
 */
float lis2dw12_get_temperature_sample_degC(void)
{
  float f32convertedTemperatureDegC = 0.0f;

  int16_t i16temperatureFromTwosComplement = convertTwosCompToInt(lis2dw12_get_temperature_sample_raw());

  i16temperatureFromTwosComplement >>= 4;

  f32convertedTemperatureDegC = (((float)i16temperatureFromTwosComplement / LSB_PER_DEG_C ) + LIS2DW12_TEMPERATURE_OFFSET_DEG_C);

  return f32convertedTemperatureDegC;
}


uint16_t lis2dw12_get_temperature_sample_raw(void)
{
  uint16_t u16tempCombined = (get_temperature_MSB() << 8) | get_temperature_LSB();

  return u16tempCombined;
}


static uint8_t get_temperature_MSB(void)
{
  uint8_t u8MSBTemp = 0;
  uint8_t au8cmd[1];
  au8cmd[0] = (READ_CMD_BIT_MASK | OUT_TEMPERATURE_H_RO);

  drv_SPI_assertCS(FALSE);
  drv_SPI_transmitReceive(au8cmd, &u8MSBTemp, sizeof(au8cmd), sizeof(u8MSBTemp));
  drv_SPI_assertCS(TRUE);

  return u8MSBTemp;
}

static uint8_t get_temperature_LSB(void)
{
  uint8_t u8LSBTemp = 0;
  uint8_t au8cmd[1];
  au8cmd[0] = (READ_CMD_BIT_MASK | OUT_TEMPERATURE_L_RO);

  drv_SPI_assertCS(FALSE);
  drv_SPI_transmitReceive(au8cmd, &u8LSBTemp, sizeof(au8cmd), sizeof(u8LSBTemp));
  drv_SPI_assertCS(TRUE);

  return u8LSBTemp;
}


static int16_t convertTwosCompToInt(uint16_t u16twoscomp)
{
  int16_t y = (u16twoscomp ^ 0xFFFF) + 1; // Invert bits and add 1
  if (u16twoscomp & (1 << 14)) // Check if 14th bit is set (meaning signed)
  {
    return (~(y) + 1); // invert back from unsigned to signed.
  }

  return u16twoscomp; // if no sign bit then fine...
}

/* Global variables ---------------------------------------------------*/

/* Public functions ----------------------------------------------------*/

/* Private functions ---------------------------------------------------*/

/******************* (C) COPYRIGHT 2023*****END OF FILE****/
