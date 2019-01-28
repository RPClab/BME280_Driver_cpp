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
 * @file	bme280.h
 * @date	14 Feb 2018
 * @version	3.3.4
 * @brief
 *
 */
/*! @file bme280.h
    @brief Sensor driver for BME280 sensor */
/*!
 * @defgroup BME280 SENSOR API
 * @{*/
#ifndef BME280_H_
#define BME280_H_
#include "IO.hpp"
/* Header includes */
#include "bme280_defs.hpp"
#include <map>
#include <iostream>
#include "data.hpp"
class settings
{
public:
    void setOversamplingPressure(const std::string& over)
    {
        std::map<std::string,uint8_t>::iterator found=Oversampling.find(over);
        if(found!=Oversampling.end()) osr_p=found->second;
        else
        {
            std::cout<<"Unknowm Oversampling option"<<std::endl;
        }
    }
    void setOversamplingTemperature(const std::string& over)
    {
        std::map<std::string,uint8_t>::iterator found=Oversampling.find(over);
        if(found!=Oversampling.end()) osr_t=found->second;
        else
        {
            std::cout<<"Unknowm Oversampling option"<<std::endl;
        }
    }
    void setOversamplingHumidity(const std::string& over)
    {
        std::map<std::string,uint8_t>::iterator found=Oversampling.find(over);
        if(found!=Oversampling.end()) osr_h=found->second;
        else
        {
            std::cout<<"Unknowm Oversampling option"<<std::endl;
        }
    }
    void setStandbyTime(const std::string& time)
    {
        std::map<std::string,uint8_t>::iterator found=StandbyTime.find(time);
        if(found!=Oversampling.end()) standby_time=found->second;
        else
        {
            std::cout<<"Unknowm StandbyTimne option"<<std::endl;
        }
    }
    void setFilterCoefficient(const std::string& coeff)
    {
        std::map<std::string,uint8_t>::iterator found=FilterCoefficient.find(coeff);
        if(found!=Oversampling.end()) standby_time=found->second;
        else
        {
            std::cout<<"Unknowm FilterCoefficient option"<<std::endl;
        }
    }
    uint8_t getOversamplingPressure()
    {
        return osr_p;
    }
    uint8_t getOversamplingTemperature()
    {
        return osr_t;
    }
    uint8_t getOversamplingHumidity()
    {
        return osr_h;
    }
    uint8_t getFilterCoefficient()
    {
        return filter;
    }
    uint8_t getStandbyTime()
    {
        return standby_time;
    }
    
    void setOversamplingPressure(uint8_t val)
    {
        osr_p=val;
    }
    void setOversamplingTemperature(uint8_t val)
    {
        osr_t=val;
    }
    void setOversamplingHumidity(uint8_t val)
    {
        osr_h=val;
    }
    void setFilterCoefficient(uint8_t val)
    {
        filter=val;
    }
    void setStandbyTime(uint8_t val)
    {
        standby_time=val;
    }
private:
    /*! pressure oversampling */
	uint8_t osr_p;
	/*! temperature oversampling */
	uint8_t osr_t;
	/*! humidity oversampling */
	uint8_t osr_h;
	/*! filter coefficient */
	uint8_t filter;
	/*! standby time */
	uint8_t standby_time;
    /**\name Oversampling macros **/
    std::map<std::string,uint8_t> Oversampling{{"NO_OVERSAMPLING",0x00},{"1X",0x01},{"2X",0x02},{"4X",0x03},{"8X",0x04},{"16X",0x05}};
    std::map<std::string,uint8_t> StandbyTime{{"1ms",0x00},{"62.5ms",0x01},{"125ms",0x02},{"250ms",0x03},{"500ms",0x04},{"1000ms",0x05},{"10ms",0x06},{"20ms",0x07}};
    std::map<std::string,uint8_t> FilterCoefficient{{"OFF",0x00},{"2",0x01},{"4",0x02},{"8",0x03},{"16",0x04}};
};


class bme280
{
public:
    bme280(IO& io,const settings& settings):m_IO(&io),m_settings(settings){};
    void delay_ms(uint32_t period);
    data& getData()
    {
        return m_data;
    }
    /*!
    *  @brief This API is the entry point.
    *  It reads the chip-id and calibration data from the sensor.
    *
    *  @return Result of API execution status
    *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t init();

    /*!
    * @brief This API writes the given data to the register address
    * of the sensor.
    *
    * @param[in] reg_addr : Register address from where the data to be written.
    * @param[in] reg_data : Pointer to data buffer which is to be written
    * in the sensor.
    * @param[in] len : No of bytes of data to write..
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len);

    /*!
    * @brief This API reads the data from the given register address of the sensor.
    *
    * @param[in] reg_addr : Register address from where the data to be read
    * @param[out] reg_data : Pointer to data buffer to store the read data.
    * @param[in] len : No of bytes of data to be read.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

    /*!
    * @brief This API sets the oversampling, filter and standby duration
    * (normal mode) settings in the sensor.
    *
    * @param[in] desired_settings : Variable used to select the settings which
    * are to be set in the sensor.
    *
    * @note : Below are the macros to be used by the user for selecting the
    * desired settings. User can do OR operation of these macros for configuring
    * multiple settings.
    *
    * Macros		  |   Functionality
    * -----------------------|----------------------------------------------
    * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
    * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
    * BME280_OSR_HUM_SEL    |   To set humidity oversampling.
    * BME280_FILTER_SEL     |   To set filter setting.
    * BME280_STANDBY_SEL  |   To set standby duration setting.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
    */
    int8_t set_sensor_settings(uint8_t desired_settings);

    /*!
    * @brief This API gets the oversampling, filter and standby duration
    * (normal mode) settings from the sensor.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
    */
    int8_t get_sensor_settings();

    /*!
    * @brief This API sets the power mode of the sensor.
    *
    * @param[in] sensor_mode : Variable which contains the power mode to be set.
    *
    *    sensor_mode           |   Macros
    * ---------------------|-------------------
    *     0                | BME280_SLEEP_MODE
    *     1                | BME280_FORCED_MODE
    *     3                | BME280_NORMAL_MODE
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t set_sensor_mode(uint8_t sensor_mode);

    /*!
    * @brief This API gets the power mode of the sensor.
    *
    * @param[out] sensor_mode : Pointer variable to store the power mode.
    *
    *   sensor_mode            |   Macros
    * ---------------------|-------------------
    *     0                | BME280_SLEEP_MODE
    *     1                | BME280_FORCED_MODE
    *     3                | BME280_NORMAL_MODE
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t get_sensor_mode(uint8_t *sensor_mode);

    /*!
    * @brief This API performs the soft reset of the sensor.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
    */
    int8_t soft_reset();

    /*!
    * @brief This API reads the pressure, temperature and humidity data from the
    * sensor, compensates the data and store it in the bme280_data structure
    * instance passed by the user.
    *
    * @param[in] sensor_comp : Variable which selects which data to be read from
    * the sensor.
    *
    * sensor_comp |   Macros
    * ------------|-------------------
    *     1       | BME280_PRESS
    *     2       | BME280_TEMP
    *     4       | BME280_HUM
    *     7       | BME280_ALL
    *
    * @param[out] comp_data : Structure instance of bme280_data.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t get_sensor_data(uint8_t sensor_comp);

    /*!
    *  @brief This API is used to parse the pressure, temperature and
    *  humidity data and store it in the bme280_uncomp_data structure instance.
    *
    *  @param[in] reg_data     : Contains register data which needs to be parsed
    *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
    *  and humidity data.
    */
    void parse_sensor_data(const uint8_t *reg_data);

    /*!
    * @brief This API is used to compensate the pressure and/or
    * temperature and/or humidity data according to the component selected by the
    * user.
    *
    * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
    * humidity.
    * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
    * humidity data.
    * @param[out] comp_data : Contains the compensated pressure and/or temperature
    * and/or humidity data.
    *
    * @return Result of API execution status.
    * @retval zero -> Success / -ve value -> Error
    */
    int8_t compensate_data(uint8_t sensor_comp);
    IO* m_IO{nullptr};
    /*! Sensor settings */
	settings m_settings;
private:
    
    /*! Chip Id */
	uint8_t m_chip_id;
    data m_data;
    /*!
    * @brief This internal API puts the device to sleep mode.
    *
    * @return Result of API execution status.
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t put_device_to_sleep();

    /*!
    * @brief This internal API writes the power mode in the sensor.
    *
    * @param[in] sensor_mode : Variable which contains the power mode to be set.
    *
    * @return Result of API execution status.
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t write_power_mode(uint8_t sensor_mode);

    /*!
    * @brief This internal API interleaves the register address between the
    * register data buffer for burst write operation.
    *
    * @param[in] reg_addr : Contains the register address array.
    * @param[out] temp_buff : Contains the temporary buffer to store the
    * register data and register address.
    * @param[in] reg_data : Contains the register data to be written in the
    * temporary buffer.
    * @param[in] len : No of bytes of data to be written for burst write.
    */
    void interleave_reg_addr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);

    /*!
    * @brief This internal API reads the calibration data from the sensor, parse
    * it and store in the device structure.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t get_calib_data();

    /*!
    *  @brief This internal API is used to parse the temperature and
    *  pressure calibration data and store it in the device structure.
    *
    *  @param[in] reg_data : Contains the calibration data to be parsed.
    */
    void parse_temp_press_calib_data(const uint8_t *reg_data);

    /*!
    *  @brief This internal API is used to parse the humidity calibration data
    *  and store it in device structure.
    *
    *  @param[in] reg_data : Contains calibration data to be parsed.
    */
    void parse_humidity_calib_data(const uint8_t *reg_data);

    /*!
    * @brief This internal API is used to identify the settings which the user
    * wants to modify in the sensor.
    *
    * @param[in] sub_settings : Contains the settings subset to identify particular
    * group of settings which the user is interested to change.
    * @param[in] desired_settings : Contains the user specified settings.
    *
    * @return Indicates whether user is interested to modify the settings which
    * are related to sub_settings.
    * @retval True -> User wants to modify this group of settings
    * @retval False -> User does not want to modify this group of settings
    */
    uint8_t are_settings_changed(uint8_t sub_settings, uint8_t desired_settings);

    /*!
    * @brief This API sets the humidity oversampling settings of the sensor.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t set_osr_humidity_settings();

    /*!
    * @brief This internal API sets the oversampling settings for pressure,
    * temperature and humidity in the sensor.
    *
    * @param[in] desired_settings : Variable used to select the settings which
    * are to be set.
    * @param[in] dev : Structure instance of bme280_dev.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t set_osr_settings(uint8_t desired_settings);

    /*!
    * @brief This API sets the pressure and/or temperature oversampling settings
    * in the sensor according to the settings selected by the user.
    *
    * @param[in] desired_settings: variable to select the pressure and/or
    * temperature oversampling settings.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t set_osr_press_temp_settings(uint8_t desired_settings);

    /*!
    * @brief This internal API fills the pressure oversampling settings provided by
    * the user in the data buffer so as to write in the sensor.
    *
    * @param[out] reg_data : Variable which is filled according to the pressure
    * oversampling data provided by the user.
    */
    void fill_osr_press_settings(uint8_t *reg_data);

    /*!
    * @brief This internal API fills the temperature oversampling settings provided
    * by the user in the data buffer so as to write in the sensor.
    *
    * @param[out] reg_data : Variable which is filled according to the temperature
    * oversampling data provided by the user.
    */
    void fill_osr_temp_settings(uint8_t *reg_data);

    /*!
    * @brief This internal API sets the filter and/or standby duration settings
    * in the sensor according to the settings selected by the user.
    *
    * @param[in] desired_settings : variable to select the filter and/or
    * standby duration settings.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t set_filter_standby_settings(uint8_t desired_settings);

    /*!
    * @brief This internal API fills the filter settings provided by the user
    * in the data buffer so as to write in the sensor.
    *
    * @param[out] reg_data : Variable which is filled according to the filter
    * settings data provided by the user.
    */
    void fill_filter_settings(uint8_t *reg_data);

    /*!
    * @brief This internal API fills the standby duration settings provided by the
    * user in the data buffer so as to write in the sensor.
    *
    * @param[out] reg_data : Variable which is filled according to the standby
    * settings data provided by the user.
    */
    void fill_standby_settings(uint8_t *reg_data);

    /*!
    * @brief This internal API parse the oversampling(pressure, temperature
    * and humidity), filter and standby duration settings and store in the
    * device structure.
    *
    * @param[in] reg_data : Register data to be parsed.
    */
    void parse_device_settings(const uint8_t *reg_data);

    /*!
    * @brief This internal API reloads the already existing device settings in the
    * sensor after soft reset.
    *
    * @param[in] settings : Pointer variable which contains the settings to
    * be set in the sensor.
    *
    * @return Result of API execution status
    * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
    */
    int8_t reload_device_settings();
};
#endif /* BME280_H_ */
/** @}*/
