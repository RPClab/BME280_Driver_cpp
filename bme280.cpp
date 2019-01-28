/**\mainpage
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
 * File		bme280.c
 * Date		14 Feb 2018
 * Version	3.3.4
 *
 */

/*! @file bme280.c
    @brief Sensor driver for BME280 sensor */
#include "bme280.hpp"


void bme280::delay_ms(uint32_t period)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(period));
}

/**\name Internal macros */
/* To identify osr settings selected by user */
#define OVERSAMPLING_SETTINGS		UINT8_C(0x07)
/* To identify filter and standby settings selected by user */
#define FILTER_STANDBY_SETTINGS		UINT8_C(0x18)


/****************** Global Function Definitions *******************************/

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 */
int8_t bme280::bme280_init()
{
	int8_t rslt;
	/* chip id read try count */
	uint8_t try_count = 5;
	uint8_t chip_id = 0;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();
	/* Proceed if null check is fine */
	if (rslt ==  BME280_OK) {
		while (try_count) {
			/* Read the chip-id of bme280 sensor */
			rslt = bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1);
			/* Check for chip id validity */
			if ((rslt == BME280_OK) && (chip_id == BME280_CHIP_ID)) {
				m_chip_id = chip_id;
				/* Reset the sensor */
				rslt = bme280_soft_reset();
				if (rslt == BME280_OK) {
					/* Read the calibration data */
					rslt = get_calib_data();
				}
				break;
			}
			/* Wait for 1 ms */
			delay_ms(1);
			--try_count;
		}
		/* Chip id check failed */
		if (!try_count)
			rslt = BME280_E_DEV_NOT_FOUND;
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t bme280::bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();
	/* Proceed if null check is fine */
	if (rslt == BME280_OK) {
		/* If interface selected is SPI */
		if (m_IO->getInterfaceName()!= "I2C")
			reg_addr = reg_addr | 0x80;
		/* Read the data  */
		rslt = m_IO->read(reg_addr, reg_data, len);
		/* Check for communication error */
		if (rslt != BME280_OK)
			rslt = BME280_E_COMM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bme280::bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
	int8_t rslt;
	uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */

	if (len > 10)
		len = 10;

	uint16_t temp_len;
	uint8_t reg_addr_cnt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();
	/* Check for arguments validity */
	if ((rslt ==  BME280_OK) && (reg_addr != nullptr) && (reg_data != nullptr)) {
		if (len != 0) {
			temp_buff[0] = reg_data[0];
			/* If interface selected is SPI */
			if (m_IO->getInterfaceName() !="I2C") {
				for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)
					reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
			}
			/* Burst write mode */
			if (len > 1) {
				/* Interleave register address w.r.t data for
				burst write*/
				interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
				temp_len = ((len * 2) - 1);
			} else {
				temp_len = len;
			}
			rslt = m_IO->write(reg_addr[0], temp_buff, temp_len);
			/* Check for communication error */
			if (rslt != BME280_OK)
				rslt = BME280_E_COMM_FAIL;
		} else {
			rslt = BME280_E_INVALID_LEN;
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}


	return rslt;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
int8_t bme280::bme280_set_sensor_settings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t sensor_mode;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();
	/* Proceed if null check is fine */
	if (rslt == BME280_OK) {
		rslt = bme280_get_sensor_mode(&sensor_mode);
		if ((rslt == BME280_OK) && (sensor_mode != BME280_SLEEP_MODE))
			rslt = put_device_to_sleep();
		if (rslt == BME280_OK) {
			/* Check if user wants to change oversampling
			   settings */
			if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings))
				rslt = set_osr_settings(desired_settings);
			/* Check if user wants to change filter and/or
			   standby settings */
			if ((rslt == BME280_OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))
				rslt = set_filter_standby_settings(desired_settings);
		}
	}

	return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
int8_t bme280::bme280_get_sensor_settings()
{
	int8_t rslt;
	uint8_t reg_data[4];

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();
	/* Proceed if null check is fine */
	if (rslt == BME280_OK) {
		rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4);
		if (rslt == BME280_OK)
			parse_device_settings(reg_data);
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bme280::bme280_set_sensor_mode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t last_set_mode;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();

	if (rslt == BME280_OK) {
		rslt = bme280_get_sensor_mode(&last_set_mode);
		/* If the sensor is not in sleep mode put the device to sleep
		   mode */
		if ((rslt == BME280_OK) && (last_set_mode != BME280_SLEEP_MODE))
			rslt = put_device_to_sleep();
		/* Set the power mode */
		if (rslt == BME280_OK)
			rslt = write_power_mode(sensor_mode);
	}

	return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int8_t bme280::bme280_get_sensor_mode(uint8_t *sensor_mode)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();

	if (rslt == BME280_OK) {
		/* Read the power mode register */
		rslt = bme280_get_regs(BME280_PWR_CTRL_ADDR, sensor_mode, 1);
		/* Assign the power mode in the device structure */
		*sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
	}

	return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bme280::bme280_soft_reset()
{
	int8_t rslt;
	uint8_t reg_addr = BME280_RESET_ADDR;
	/* 0xB6 is the soft reset command */
	uint8_t soft_rst_cmd = 0xB6;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();
	/* Proceed if null check is fine */
	if (rslt == BME280_OK) {
		/* Write the soft reset command in the sensor */
		rslt = bme280_set_regs(&reg_addr, &soft_rst_cmd, 1);
		/* As per data sheet, startup time is 2 ms. */
		delay_ms(2);
	}

	return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t bme280::bme280_get_sensor_data(uint8_t sensor_comp)
{
	int8_t rslt;
	/* Array to store the pressure, temperature and humidity data read from
	the sensor */
	uint8_t reg_data[BME280_P_T_H_DATA_LEN] = {0};
	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check();

	if ((rslt == BME280_OK)) 
    {
		/* Read the pressure and temperature data from the sensor */
		rslt = bme280_get_regs(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN);

		if (rslt == BME280_OK) {
			/* Parse the read data from the sensor */
			bme280_parse_sensor_data(reg_data);
			/* Compensate the pressure and/or temperature and/or
			   humidity data from the sensor */
			rslt = bme280_compensate_data(sensor_comp);
		}
	} 
	else 
    {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void bme280::bme280_parse_sensor_data(const uint8_t *reg_data)
{
    m_data.resetUncompensated();
    m_data.setUncompPressure(reg_data[0],reg_data[1],reg_data[2]);
    m_data.setUncompTemperature(reg_data[3],reg_data[4],reg_data[5]);
    m_data.setUncompHumidity(reg_data[6],reg_data[7]);
}


/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t bme280::bme280_compensate_data(uint8_t sensor_comp)
{
	int8_t rslt = BME280_OK;
    
    /* Initialize to zero */
    m_data.resetCompensated();
    /* If pressure or temperature component is selected */
    if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) 
    {
        /* Compensate the temperature data */
        m_data.compensate_temperature();
    }
    if (sensor_comp & BME280_PRESS) 
    {
        /* Compensate the pressure data */
        m_data.compensate_pressure();
    }
    if (sensor_comp & BME280_HUM) 
    {
        /* Compensate the humidity data */
        m_data.compensate_humidity();
    }
	return rslt;
}

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
int8_t bme280::set_osr_settings(uint8_t desired_settings)
{
	int8_t rslt = BME280_W_INVALID_OSR_MACRO;

	if (desired_settings & BME280_OSR_HUM_SEL)
		rslt = set_osr_humidity_settings();
	if (desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL))
		rslt = set_osr_press_temp_settings(desired_settings);

	return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
int8_t bme280::set_osr_humidity_settings()
{
	int8_t rslt;
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t reg_addr = BME280_CTRL_HUM_ADDR;

	ctrl_hum = m_settings.getOversamplingHumidity() & BME280_CTRL_HUM_MSK;
	/* Write the humidity control value in the register */
	rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1);
	/* Humidity related changes will be only effective after a
	   write operation to ctrl_meas register */
	if (rslt == BME280_OK) {
		reg_addr = BME280_CTRL_MEAS_ADDR;
		rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1);
		if (rslt == BME280_OK)
			rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1);
	}

	return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
int8_t bme280::set_osr_press_temp_settings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_CTRL_MEAS_ADDR;
	uint8_t reg_data;

	rslt = bme280_get_regs(reg_addr, &reg_data, 1);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_OSR_PRESS_SEL)
			fill_osr_press_settings(&reg_data);
		if (desired_settings & BME280_OSR_TEMP_SEL)
			fill_osr_temp_settings(&reg_data);
		/* Write the oversampling settings in the register */
		rslt = bme280_set_regs(&reg_addr, &reg_data, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
int8_t bme280::set_filter_standby_settings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_CONFIG_ADDR;
	uint8_t reg_data;

	rslt = bme280_get_regs(reg_addr, &reg_data, 1);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_FILTER_SEL)
			fill_filter_settings(&reg_data);
		if (desired_settings & BME280_STANDBY_SEL)
			fill_standby_settings(&reg_data);
		/* Write the oversampling settings in the register */
		rslt = bme280_set_regs(&reg_addr, &reg_data, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
void bme280::fill_filter_settings(uint8_t *reg_data)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, m_settings.getFilterCoefficient());
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void bme280::fill_standby_settings(uint8_t *reg_data)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, m_settings.getStandbyTime());
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void bme280::fill_osr_press_settings(uint8_t *reg_data)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, m_settings.getOversamplingPressure());
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
void bme280::fill_osr_temp_settings(uint8_t *reg_data)
{
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, m_settings.getOversamplingTemperature());
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 */
void bme280::parse_device_settings(const uint8_t *reg_data)
{
	m_settings.setOversamplingHumidity(BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM));
	m_settings.setOversamplingPressure(BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS));
	m_settings.setOversamplingTemperature(BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP));
	m_settings.setFilterCoefficient(BME280_GET_BITS(reg_data[3], BME280_FILTER));
	m_settings.setStandbyTime(BME280_GET_BITS(reg_data[3], BME280_STANDBY));
}
/*!
 * @brief This internal API writes the power mode in the sensor.
 */
int8_t bme280::write_power_mode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_PWR_CTRL_ADDR;
	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;

	/* Read the power mode register */
	rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1);
	/* Set the power mode */
	if (rslt == BME280_OK) {
		sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);
		/* Write the power mode in the register */
		rslt = bme280_set_regs(&reg_addr, &sensor_mode_reg_val, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
int8_t bme280::put_device_to_sleep()
{
	int8_t rslt;
	uint8_t reg_data[4];
	struct bme280_settings settings;

	rslt = bme280_get_regs(BME280_CTRL_HUM_ADDR, reg_data, 4);
	if (rslt == BME280_OK) {
		parse_device_settings(reg_data);
		rslt = bme280_soft_reset();
		if (rslt == BME280_OK)
			rslt = reload_device_settings();
	}

	return rslt;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
int8_t bme280::reload_device_settings()
{
	int8_t rslt;

	rslt = set_osr_settings(BME280_ALL_SETTINGS_SEL);
	if (rslt == BME280_OK)
		rslt = set_filter_standby_settings(BME280_ALL_SETTINGS_SEL);

	return rslt;
}


/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
int8_t bme280::get_calib_data()
{
	int8_t rslt;
	uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;
	/* Array to store calibration data */
	uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = {0};

	/* Read the calibration data from the sensor */
	rslt = bme280_get_regs(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);

	if (rslt == BME280_OK) {
		/* Parse temperature and pressure calibration data and store
		   it in device structure */
		parse_temp_press_calib_data(calib_data);

		reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;
		/* Read the humidity calibration data from the sensor */
		rslt = bme280_get_regs(reg_addr, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN);
		if (rslt == BME280_OK) {
			/* Parse humidity calibration data and store it in
			   device structure */
			parse_humidity_calib_data(calib_data);
		}
	}

	return rslt;
}

/*!
 * @brief This internal API interleaves the register address between the
 * register data buffer for burst write operation.
 */
void bme280::interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
	uint8_t index;

	for (index = 1; index < len; index++) {
		temp_buff[(index * 2) - 1] = reg_addr[index];
		temp_buff[index * 2] = reg_data[index];
	}
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
void bme280::parse_temp_press_calib_data(const uint8_t *reg_data)
{
	m_data.returnCalibData().m_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	m_data.returnCalibData().m_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	m_data.returnCalibData().m_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	m_data.returnCalibData().m_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	m_data.returnCalibData().m_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	m_data.returnCalibData().m_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	m_data.returnCalibData().m_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	m_data.returnCalibData().m_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	m_data.returnCalibData().m_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	m_data.returnCalibData().m_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	m_data.returnCalibData().m_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	m_data.returnCalibData().m_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	m_data.returnCalibData().m_H1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
void bme280::parse_humidity_calib_data(const uint8_t *reg_data)
{
	int16_t m_H4_lsb{0};
	int16_t m_H4_msb{0};
	int16_t m_H5_lsb{0};
	int16_t m_H5_msb{0};
	m_data.returnCalibData().m_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	m_data.returnCalibData().m_H3 = reg_data[2];
	m_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	m_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
	m_data.returnCalibData().m_H4 = m_H4_msb | m_H4_lsb;
	m_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	m_H5_lsb = (int16_t)(reg_data[4] >> 4);
	m_data.returnCalibData().m_H5 = m_H5_msb | m_H5_lsb;
	m_data.returnCalibData().m_H6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
uint8_t bme280::are_settings_changed(uint8_t sub_settings, uint8_t desired_settings)
{
	uint8_t settings_changed = false;

	if (sub_settings & desired_settings) {
		/* User wants to modify this particular settings */
		settings_changed = true;
	} else {
		/* User don't want to modify this particular settings */
		settings_changed = true;
	}

	return settings_changed;
}

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
int8_t bme280::null_ptr_check()
{
	int8_t rslt;

	if (m_IO == nullptr) {
		/* Device structure pointer is not valid */
		rslt = BME280_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BME280_OK;
	}

	return rslt;
}
