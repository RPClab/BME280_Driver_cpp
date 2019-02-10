/*! @file bme280.c
    @brief Sensor driver for BME280 sensor */
#include "bme280.hpp"
#include <chrono>
#include <thread>

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
int8_t bme280::init()
{
	int8_t rslt;
	/* chip id read try count */
	uint8_t try_count = 5;
	uint8_t chip_id = 0;
    while (try_count) 
    {
			/* Read the chip-id of bme280 sensor */
			rslt = get_regs(CHIP_ID_ADDR, &chip_id, 1);
			/* Check for chip id validity */
			if ((rslt == OK) && (chip_id == CHIP_ID)) 
            {
				m_chip_id = chip_id;
				/* Reset the sensor */
				rslt = soft_reset();
				if (rslt == OK) 
                {
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
    if (!try_count) rslt = E_DEV_NOT_FOUND;
	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t bme280::get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt;
    /* If interface selected is SPI */
    if (m_IO->getInterfaceName()!= "I2C") reg_addr = reg_addr | 0x80;
    /* Read the data  */
    rslt = m_IO->read(reg_addr, reg_data, len);
    /* Check for communication error */
    if (rslt != OK) rslt = E_COMM_FAIL;
	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bme280::set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
	int8_t rslt;
	uint8_t temp_buff[20]; /* Typically not to write more than 10 registers */
	if (len > 10) len = 10;
	uint16_t temp_len;
	uint8_t reg_addr_cnt;
    if (len != 0) 
    {
        temp_buff[0] = reg_data[0];
        /* If interface selected is SPI */
        if (m_IO->getInterfaceName() !="I2C") 
        {
            for (reg_addr_cnt = 0; reg_addr_cnt < len; reg_addr_cnt++)reg_addr[reg_addr_cnt] = reg_addr[reg_addr_cnt] & 0x7F;
        }
        /* Burst write mode */
        if (len > 1) 
        {
            /* Interleave register address w.r.t data for burst write*/
            interleave_reg_addr(reg_addr, temp_buff, reg_data, len);
            temp_len = ((len * 2) - 1);
        } 
        else
        {
            temp_len = len;
        }
        rslt = m_IO->write(reg_addr[0], temp_buff, temp_len);
        /* Check for communication error */
        if (rslt != OK) rslt = E_COMM_FAIL;
    } 
    else 
    {
        rslt = E_INVALID_LEN;
    }
	return rslt;
}

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 */
int8_t bme280::set_sensor_settings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t sensor_mode;
    rslt = get_sensor_mode(&sensor_mode);
    if ((rslt == OK) && (sensor_mode != SLEEP_MODE))rslt = put_device_to_sleep();
    if (rslt == OK) 
    {
        /* Check if user wants to change oversampling settings */
        if (are_settings_changed(OVERSAMPLING_SETTINGS, desired_settings)) rslt = set_osr_settings(desired_settings);
        /* Check if user wants to change filter and/or standby settings */
        if ((rslt == OK) && are_settings_changed(FILTER_STANDBY_SETTINGS, desired_settings))rslt = set_filter_standby_settings(desired_settings);
    }
	return rslt;
}

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 */
int8_t bme280::get_sensor_settings()
{
	int8_t rslt;
	uint8_t reg_data[4];
    rslt = get_regs(CTRL_HUM_ADDR, reg_data, 4);
    if (rslt == OK)parse_device_settings(reg_data);
	return rslt;
}

/*!
 * @brief This API sets the power mode of the sensor.
 */
int8_t bme280::set_sensor_mode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t last_set_mode;
    rslt = get_sensor_mode(&last_set_mode);
    /* If the sensor is not in sleep mode put the device to sleep mode */
    if ((rslt == OK) && (last_set_mode != SLEEP_MODE))rslt = put_device_to_sleep();
    /* Set the power mode */
    if (rslt == OK)rslt = write_power_mode(sensor_mode);
	return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
int8_t bme280::get_sensor_mode(uint8_t *sensor_mode)
{
	int8_t rslt;
    /* Read the power mode register */
    rslt = get_regs(PWR_CTRL_ADDR, sensor_mode, 1);
    /* Assign the power mode in the device structure */
    *sensor_mode = GET_BITS_POS_0(*sensor_mode,SENSOR_MODE);
	return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bme280::soft_reset()
{
	int8_t rslt;
	uint8_t reg_addr = RESET_ADDR;
	/* 0xB6 is the soft reset command */
	uint8_t soft_rst_cmd = 0xB6;
    /* Write the soft reset command in the sensor */
    rslt = set_regs(&reg_addr, &soft_rst_cmd, 1);
    /* As per data sheet, startup time is 2 ms. */
    delay_ms(2);
	return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t bme280::get_sensor_data(uint8_t sensor_comp)
{
	int8_t rslt;
	/* Array to store the pressure, temperature and humidity data read from
	the sensor */
	uint8_t reg_data[P_T_H_DATA_LEN] = {0};
    /* Read the pressure and temperature data from the sensor */
    rslt = get_regs(DATA_ADDR, reg_data,P_T_H_DATA_LEN);
    if (rslt == OK) 
    {
        /* Parse the read data from the sensor */
        parse_sensor_data(reg_data);
        /* Compensate the pressure and/or temperature and/or humidity data from the sensor */
        rslt = compensate_data(sensor_comp);
    }
	return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void bme280::parse_sensor_data(const uint8_t *reg_data)
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
int8_t bme280::compensate_data(uint8_t sensor_comp)
{
	int8_t rslt = OK;
    /* Initialize to zero */
    m_data.resetCompensated();
    /* If pressure or temperature component is selected */
    if (sensor_comp & (PRESS | TEMP | HUM)) 
    {
        /* Compensate the temperature data */
        m_data.compensate_temperature();
    }
    if (sensor_comp & PRESS) 
    {
        /* Compensate the pressure data */
        m_data.compensate_pressure();
    }
    if (sensor_comp & HUM) 
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
	int8_t rslt = W_INVALID_OSR_MACRO;
	if (desired_settings & OSR_HUM_SEL) rslt = set_osr_humidity_settings();
	if (desired_settings & (OSR_PRESS_SEL | OSR_TEMP_SEL))rslt = set_osr_press_temp_settings(desired_settings);
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
	uint8_t reg_addr = CTRL_HUM_ADDR;
	ctrl_hum = m_settings.getOversamplingHumidity() & CTRL_HUM_MSK;
	/* Write the humidity control value in the register */
	rslt = set_regs(&reg_addr, &ctrl_hum, 1);
	/* Humidity related changes will be only effective after a
	   write operation to ctrl_meas register */
	if (rslt == OK) 
    {
		reg_addr = CTRL_MEAS_ADDR;
		rslt = get_regs(reg_addr, &ctrl_meas, 1);
		if (rslt == OK)rslt = set_regs(&reg_addr, &ctrl_meas, 1);
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
	uint8_t reg_addr = CTRL_MEAS_ADDR;
	uint8_t reg_data;
	rslt = get_regs(reg_addr, &reg_data, 1);
	if (rslt == OK) 
    {
		if (desired_settings & OSR_PRESS_SEL)fill_osr_press_settings(&reg_data);
		if (desired_settings & OSR_TEMP_SEL)fill_osr_temp_settings(&reg_data);
		/* Write the oversampling settings in the register */
		rslt = set_regs(&reg_addr, &reg_data, 1);
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
	uint8_t reg_addr = CONFIG_ADDR;
	uint8_t reg_data;
	rslt = get_regs(reg_addr, &reg_data, 1);
	if (rslt == OK) 
    {
		if (desired_settings & FILTER_SEL)fill_filter_settings(&reg_data);
		if (desired_settings & STANDBY_SEL)fill_standby_settings(&reg_data);
		/* Write the oversampling settings in the register */
		rslt = set_regs(&reg_addr, &reg_data, 1);
	}
	return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
void bme280::fill_filter_settings(uint8_t *reg_data)
{
	*reg_data = SET_BITS(*reg_data, FILTER, m_settings.getFilterCoefficient());
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void bme280::fill_standby_settings(uint8_t *reg_data)
{
	*reg_data = SET_BITS(*reg_data, STANDBY, m_settings.getStandbyTime());
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void bme280::fill_osr_press_settings(uint8_t *reg_data)
{
	*reg_data = SET_BITS(*reg_data, CTRL_PRESS, m_settings.getOversamplingPressure());
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
void bme280::fill_osr_temp_settings(uint8_t *reg_data)
{
	*reg_data = SET_BITS(*reg_data, CTRL_TEMP, m_settings.getOversamplingTemperature());
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 */
void bme280::parse_device_settings(const uint8_t *reg_data)
{
	m_settings.setOversamplingHumidity(GET_BITS_POS_0(reg_data[0], CTRL_HUM));
	m_settings.setOversamplingPressure(GET_BITS(reg_data[2], CTRL_PRESS));
	m_settings.setOversamplingTemperature(GET_BITS(reg_data[2], CTRL_TEMP));
	m_settings.setFilterCoefficient(GET_BITS(reg_data[3], FILTER));
	m_settings.setStandbyTime(GET_BITS(reg_data[3], STANDBY));
}
/*!
 * @brief This internal API writes the power mode in the sensor.
 */
int8_t bme280::write_power_mode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t reg_addr = PWR_CTRL_ADDR;
	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;
	/* Read the power mode register */
	rslt = get_regs(reg_addr, &sensor_mode_reg_val, 1);
	/* Set the power mode */
	if (rslt == OK) 
    {
		sensor_mode_reg_val = SET_BITS_POS_0(sensor_mode_reg_val, SENSOR_MODE, sensor_mode);
		/* Write the power mode in the register */
		rslt = set_regs(&reg_addr, &sensor_mode_reg_val, 1);
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
	rslt = get_regs(CTRL_HUM_ADDR, reg_data, 4);
	if (rslt == OK) 
    {
		parse_device_settings(reg_data);
		rslt = soft_reset();
		if (rslt == OK)rslt = reload_device_settings();
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
	rslt = set_osr_settings(ALL_SETTINGS_SEL);
	if (rslt == OK)rslt = set_filter_standby_settings(ALL_SETTINGS_SEL);
	return rslt;
}


/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
int8_t bme280::get_calib_data()
{
	int8_t rslt;
	uint8_t reg_addr = TEMP_PRESS_CALIB_DATA_ADDR;
	/* Array to store calibration data */
	uint8_t calib_data[TEMP_PRESS_CALIB_DATA_LEN] = {0};
	/* Read the calibration data from the sensor */
	rslt = get_regs(reg_addr, calib_data,TEMP_PRESS_CALIB_DATA_LEN);
	if (rslt == OK)
    {
		/* Parse temperature and pressure calibration data and store it in device structure */
		parse_temp_press_calib_data(calib_data);
		reg_addr = HUMIDITY_CALIB_DATA_ADDR;
		/* Read the humidity calibration data from the sensor */
		rslt = get_regs(reg_addr, calib_data, HUMIDITY_CALIB_DATA_LEN);
		if (rslt == OK) 
        {
			/* Parse humidity calibration data and store it in device structure */
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
	for (index = 1; index < len; index++) 
    {
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
	m_data.returnCalibData().m_T1 = CONCAT_BYTES(reg_data[1], reg_data[0]);
	m_data.returnCalibData().m_T2 = (int16_t)CONCAT_BYTES(reg_data[3], reg_data[2]);
	m_data.returnCalibData().m_T3 = (int16_t)CONCAT_BYTES(reg_data[5], reg_data[4]);
	m_data.returnCalibData().m_P1 = CONCAT_BYTES(reg_data[7], reg_data[6]);
	m_data.returnCalibData().m_P2 = (int16_t)CONCAT_BYTES(reg_data[9], reg_data[8]);
	m_data.returnCalibData().m_P3 = (int16_t)CONCAT_BYTES(reg_data[11], reg_data[10]);
	m_data.returnCalibData().m_P4 = (int16_t)CONCAT_BYTES(reg_data[13], reg_data[12]);
	m_data.returnCalibData().m_P5 = (int16_t)CONCAT_BYTES(reg_data[15], reg_data[14]);
	m_data.returnCalibData().m_P6 = (int16_t)CONCAT_BYTES(reg_data[17], reg_data[16]);
	m_data.returnCalibData().m_P7 = (int16_t)CONCAT_BYTES(reg_data[19], reg_data[18]);
	m_data.returnCalibData().m_P8 = (int16_t)CONCAT_BYTES(reg_data[21], reg_data[20]);
	m_data.returnCalibData().m_P9 = (int16_t)CONCAT_BYTES(reg_data[23], reg_data[22]);
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
	m_data.returnCalibData().m_H2 = (int16_t)CONCAT_BYTES(reg_data[1], reg_data[0]);
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
	if (sub_settings & desired_settings) 
    {
		/* User wants to modify this particular settings */
		settings_changed = true;
	} 
	else 
    {
		/* User don't want to modify this particular settings */
		settings_changed = true;
	}
	return settings_changed;
}
