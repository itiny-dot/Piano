/*!
 *  @file Adafruit_MPR121.h
 *
 *  This is a library for the MPR121 12-Channel Capacitive Sensor
 *
 *  Designed specifically to work with the MPR121 board.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/1982
 *
 *  These sensors use I2C to communicate, 2+ pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

// ported over to C for STM32 stm32f3xx
#ifndef SRC_ADAFRUIT_MPR121_STM32_H_
#define SRC_ADAFRUIT_MPR121_STM32_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"

// The default I2C address
#define MPR121_I2CADDR_DEFAULT 0x5A        // default I2C address
#define MPR121_TOUCH_THRESHOLD_DEFAULT 12  // default touch threshold value
#define MPR121_RELEASE_THRESHOLD_DEFAULT 6 // default release threshold value

/*!
 *  Device register map
 */
typedef enum {
	MPR121_TOUCHSTATUS_L = 0x00,
	MPR121_TOUCHSTATUS_H = 0x01,
	MPR121_FILTDATA_0L = 0x04,
	MPR121_FILTDATA_0H = 0x05,
	MPR121_BASELINE_0 = 0x1E,
	MPR121_MHDR = 0x2B,
	MPR121_NHDR = 0x2C,
	MPR121_NCLR = 0x2D,
	MPR121_FDLR = 0x2E,
	MPR121_MHDF = 0x2F,
	MPR121_NHDF = 0x30,
	MPR121_NCLF = 0x31,
	MPR121_FDLF = 0x32,
	MPR121_NHDT = 0x33,
	MPR121_NCLT = 0x34,
	MPR121_FDLT = 0x35,

	MPR121_TOUCHTH_0 = 0x41,
	MPR121_RELEASETH_0 = 0x42,
	MPR121_DEBOUNCE = 0x5B,
	MPR121_CONFIG1 = 0x5C,
	MPR121_CONFIG2 = 0x5D,
	MPR121_CHARGECURR_0 = 0x5F,
	MPR121_CHARGETIME_1 = 0x6C,
	MPR121_ECR = 0x5E,
	MPR121_AUTOCONFIG0 = 0x7B,
	MPR121_AUTOCONFIG1 = 0x7C,
	MPR121_UPLIMIT = 0x7D,
	MPR121_LOWLIMIT = 0x7E,
	MPR121_TARGETLIMIT = 0x7F,

	MPR121_GPIODIR = 0x76,
	MPR121_GPIOEN = 0x77,
	MPR121_GPIOSET = 0x78,
	MPR121_GPIOCLR = 0x79,
	MPR121_GPIOTOGGLE = 0x7A,

	MPR121_SOFTRESET = 0x80,
} MPR121_Register;

/*!
 *  @brief  Class that stores state and functions for interacting with MPR121
 *  proximity capacitive touch sensor controller.
 */
typedef struct {
	// 0x5A - 0x5D
	uint8_t address;
	I2C_HandleTypeDef *hi2c;
} Adafruit_MPR121;

// function prototypes
void Adafruit_MPR121_Init(Adafruit_MPR121 *mpr121, uint8_t address,
		I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Adafruit_MPR121_Begin(Adafruit_MPR121 *mpr121,
		uint8_t i2caddr, uint8_t touchThreshold, uint8_t releaseThreshold);
uint16_t Adafruit_MPR121_FilteredData(Adafruit_MPR121 *mpr121, uint8_t t);
uint16_t Adafruit_MPR121_BaselineData(Adafruit_MPR121 *mpr121, uint8_t t);
uint8_t Adafruit_MPR121_ReadRegister8(Adafruit_MPR121 *mpr121, uint8_t reg);
uint16_t Adafruit_MPR121_ReadRegister16(Adafruit_MPR121 *mpr121, uint8_t reg);
void Adafruit_MPR121_WriteRegister(Adafruit_MPR121 *mpr121, uint8_t reg,
		uint8_t value);
uint16_t Adafruit_MPR121_Touched(Adafruit_MPR121 *mpr121);
void Adafruit_MPR121_SetThresholds(Adafruit_MPR121 *mpr121, uint8_t touch,
		uint8_t release);

#endif /* SRC_ADAFRUIT_MPR121_STM32_H_ */
