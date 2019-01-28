/**
 * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	bme280_defs.h
 * @date	14 Feb 2018
 * @version	3.3.4
 * @brief
 *
 */

/*! @file bme280_defs.h
    @brief Sensor driver for BME280 sensor */
/*!
 * @defgroup BME280 SENSOR API
 * @brief
 * @{*/
#ifndef BME280_DEFS_H_
#define BME280_DEFS_H_

/********************************************************/
/* header includes */
#include <cstdint>

/********************************************************/
/*! @name		Common macros		        */
/********************************************************/

/**@}*/

#ifndef BME280_FLOAT_ENABLE
/* #define BME280_FLOAT_ENABLE */
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
#define BME280_64BIT_ENABLE
#endif
#endif

/**\name BME280 chip identifier */
#define CHIP_ID  uint8_t(0x60)

/**\name Register Address */
#define CHIP_ID_ADDR					uint8_t(0xD0)
#define RESET_ADDR					uint8_t(0xE0)
#define TEMP_PRESS_CALIB_DATA_ADDR	uint8_t(0x88)
#define HUMIDITY_CALIB_DATA_ADDR		uint8_t(0xE1)
#define PWR_CTRL_ADDR				uint8_t(0xF4)
#define CTRL_HUM_ADDR				uint8_t(0xF2)
#define CTRL_MEAS_ADDR				uint8_t(0xF4)
#define CONFIG_ADDR					uint8_t(0xF5)
#define DATA_ADDR					uint8_t(0xF7)

/**\name API success code */
#define OK					int8_t(0)

/**\name API error codes */
#define E_NULL_PTR			int8_t(-1)
#define E_DEV_NOT_FOUND		int8_t(-2)
#define E_INVALID_LEN		int8_t(-3)
#define E_COMM_FAIL			int8_t(-4)
#define E_SLEEP_MODE_FAIL	int8_t(-5)

/**\name API warning codes */
#define W_INVALID_OSR_MACRO      int8_t(1)

/**\name Macros related to size */
#define TEMP_PRESS_CALIB_DATA_LEN	uint8_t(26)
#define HUMIDITY_CALIB_DATA_LEN		uint8_t(7)
#define P_T_H_DATA_LEN				uint8_t(8)

/**\name Sensor power modes */
#define	SLEEP_MODE		uint8_t(0x00)
#define	FORCED_MODE		uint8_t(0x01)
#define	NORMAL_MODE		uint8_t(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#define SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))
#define SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))
#define GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define SENSOR_MODE_MSK	uint8_t(0x03)
#define SENSOR_MODE_POS	uint8_t(0x00)

#define CTRL_HUM_MSK		uint8_t(0x07)
#define CTRL_HUM_POS		uint8_t(0x00)

#define CTRL_PRESS_MSK	uint8_t(0x1C)
#define CTRL_PRESS_POS	uint8_t(0x02)

#define CTRL_TEMP_MSK	uint8_t(0xE0)
#define CTRL_TEMP_POS	uint8_t(0x05)

#define FILTER_MSK		uint8_t(0x1C)
#define FILTER_POS		uint8_t(0x02)

#define STANDBY_MSK		uint8_t(0xE0)
#define STANDBY_POS		uint8_t(0x05)

/**\name Sensor component selection macros
   These values are internal for API implementation. Don't relate this to
   data sheet.*/
#define PRESS		uint8_t(1)
#define TEMP			uint8_t(1 << 1)
#define HUM			uint8_t(1 << 2)
#define ALL			uint8_t(0x07)

/**\name Settings selection macros */
#define OSR_PRESS_SEL		uint8_t(1)
#define OSR_TEMP_SEL			uint8_t(1 << 1)
#define OSR_HUM_SEL			uint8_t(1 << 2)
#define FILTER_SEL			uint8_t(1 << 3)
#define STANDBY_SEL			uint8_t(1 << 4)
#define ALL_SETTINGS_SEL		uint8_t(0x1F)

#include <chrono>
#include <thread>
#include <iostream>
#endif /* BME280_DEFS_H_ */
/** @}*/
/** @}*/
