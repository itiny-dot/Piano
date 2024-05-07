/*!
 * @file Adafruit_MPR121.cpp
 *
 *  @mainpage Adafruit MPR121 arduino driver
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the MPR121 I2C 12-chan Capacitive Sensor
 *
 *  Designed specifically to work with the MPR121 sensor from Adafruit
 *  ----> https://www.adafruit.com/products/1982
 *
 *  These sensors use I2C to communicate, 2+ pins are required to
 *  interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 *  @section author Author
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *  @section license License
 *
 *  BSD license, all text here must be included in any redistribution.
 */

// ported over to C for STM32 stm32f3xx
#include "Adafruit_MPR121_STM32.h"

void Adafruit_MPR121_Init(Adafruit_MPR121 *mpr121, uint8_t address,
		I2C_HandleTypeDef *hi2c) {
	mpr121->address = address;
	mpr121->hi2c = hi2c;
}

/*!
 * @brief    Begin an MPR121 object on a given I2C bus. This function resets
 *           the device and writes the default settings.
 * @param    i2caddr
 *           the i2c address the device can be found on. Defaults to 0x5A.
 * @param    *theWire
 *           Wire object
 * @param    touchThreshold
 *           touch detection threshold value
 * @param    releaseThreshold
 *           release detection threshold value
 * @returns  true on success, false otherwise
 */
HAL_StatusTypeDef Adafruit_MPR121_Begin(Adafruit_MPR121 *mpr121,
		uint8_t i2caddr, uint8_t touchThreshold, uint8_t releaseThreshold) {
	mpr121->address = i2caddr;

	// Soft reset
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_SOFTRESET, 0x63);
	HAL_Delay(1);

	// Check CONFIG2 register
	uint8_t c = Adafruit_MPR121_ReadRegister8(mpr121, MPR121_CONFIG2);
	if (c != 0x24)
		return HAL_ERROR;

	// Set thresholds
	Adafruit_MPR121_SetThresholds(mpr121, touchThreshold, releaseThreshold);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_MHDR, 0x01);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_NHDR, 0x01);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_NCLR, 0x0E);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_FDLR, 0x00);

	Adafruit_MPR121_WriteRegister(mpr121, MPR121_MHDF, 0x01);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_NHDF, 0x05);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_NCLF, 0x01);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_FDLF, 0x00);

	Adafruit_MPR121_WriteRegister(mpr121, MPR121_NHDT, 0x00);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_NCLT, 0x00);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_FDLT, 0x00);

	Adafruit_MPR121_WriteRegister(mpr121, MPR121_DEBOUNCE, 0);
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_CONFIG1, 0x10); // default, 16uA charge current
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

	// Enable X electrodes and start MPR121
	uint8_t ECR_SETTING = 0x80 + 12; // 5 bits for baseline tracking & proximity disabled + X electrodes (12)
	Adafruit_MPR121_WriteRegister(mpr121, MPR121_ECR, ECR_SETTING); // start with above ECR setting

	return HAL_OK;
}

/*!
 * @brief      Set the touch and release thresholds for all 13 channels on the
 *             device to the passed values. The threshold is defined as a
 *             deviation value from the baseline value, so it remains constant
 *             even baseline value changes. Typically the touch threshold is a
 *             little bigger than the release threshold to touch debounce and
 *             hysteresis. For typical touch application, the value can be in
 *             range 0x05~0x30 for example. The setting of the threshold is
 *             depended on the actual application. For the operation details
 *             and how to set the threshold refer to application note AN3892
 *             and MPR121 design guidelines.
 * @param      touch
 *             the touch threshold value from 0 to 255.
 * @param      release
 *             the release threshold from 0 to 255.
 */
void Adafruit_MPR121_SetThresholds(Adafruit_MPR121 *mpr121, uint8_t touch,
		uint8_t release) {
	// Set all thresholds (the same)
	for (uint8_t i = 0; i < 12; i++) {
		Adafruit_MPR121_WriteRegister(mpr121, MPR121_TOUCHTH_0 + 2 * i, touch);
		Adafruit_MPR121_WriteRegister(mpr121, MPR121_RELEASETH_0 + 2 * i, release);
	}
}

/*!
 * @brief      Read the filtered data from channel t. The ADC raw data outputs
 *             run through 3 levels of digital filtering to filter out the high
 *             frequency and low frequency noise encountered. For detailed
 *             information on this filtering see page 6 of the device datasheet.
 * @param      t
 *             the channel to read
 * @returns    the filtered reading as a 10 bit unsigned value
 */
uint16_t Adafruit_MPR121_FilteredData(Adafruit_MPR121 *mpr121, uint8_t t) {
	if (!mpr121 || t > 12)
		return 0;
	return Adafruit_MPR121_ReadRegister16(mpr121, MPR121_FILTDATA_0L + t * 2);
}

/*!
 * @brief      Read the baseline value for the channel. The 3rd level filtered
 *             result is internally 10bit but only high 8 bits are readable
 *             from registers 0x1E~0x2A as the baseline value output for each
 *             channel.
 * @param      t
 *             the channel to read.
 * @returns    the baseline data that was read
 */
uint16_t Adafruit_MPR121_BaselineData(Adafruit_MPR121 *mpr121, uint8_t t) {
	if (!mpr121 || t > 12)
		return 0;
	uint16_t bl = Adafruit_MPR121_ReadRegister8(mpr121, MPR121_BASELINE_0 + t);
	return (bl << 2);
}

/*!
 * @brief      Read the touch status of all 13 channels as bit values in a 12
 *             bit integer.
 * @returns    a 12 bit integer with each bit corresponding to the touch status
 *             of a sensor. For example, if bit 0 is set then channel 0 of the
 *             device is currently deemed to be touched.
 */
uint16_t Adafruit_MPR121_Touched(Adafruit_MPR121 *mpr121) {
	if (!mpr121)
		return 0;
	uint16_t t = Adafruit_MPR121_ReadRegister16(mpr121, MPR121_TOUCHSTATUS_L);
	return t & 0x0FFF;
}

/*!
 * @brief      Read the contents of an 8 bit device register.
 * @param      reg the register address to read from
 * @returns    the 8 bit value that was read.
 */
uint8_t Adafruit_MPR121_ReadRegister8(Adafruit_MPR121 *mpr121, uint8_t reg) {
	uint8_t value = 0;
	HAL_I2C_Mem_Read(mpr121->hi2c, mpr121->address << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
	return value;
}

/*!
 * @brief      Read the contents of a 16 bit device register.
 * @param      reg the register address to read from
 * @returns    the 16 bit value that was read.
 */
uint16_t Adafruit_MPR121_ReadRegister16(Adafruit_MPR121 *mpr121, uint8_t reg) {
	uint8_t data[2];
	HAL_I2C_Mem_Read(mpr121->hi2c, mpr121->address << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
	return (uint16_t) ((data[1] << 8) | data[0]);
}

/*!
 * @brief      Writes 8-bits to the specified destination register
 * @param      reg the register address to write to
 * @param      value the value to write
 */
void Adafruit_MPR121_WriteRegister(Adafruit_MPR121 *mpr121, uint8_t reg,
		uint8_t value) {
	HAL_I2C_Mem_Write(mpr121->hi2c, mpr121->address << 1, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}
