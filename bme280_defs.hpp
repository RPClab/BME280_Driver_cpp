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

#include <string>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
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

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM	uint8_t(0x76)
#define BME280_I2C_ADDR_SEC		uint8_t(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID  uint8_t(0x60)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR					uint8_t(0xD0)
#define BME280_RESET_ADDR					uint8_t(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR	uint8_t(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR		uint8_t(0xE1)
#define BME280_PWR_CTRL_ADDR				uint8_t(0xF4)
#define BME280_CTRL_HUM_ADDR				uint8_t(0xF2)
#define BME280_CTRL_MEAS_ADDR				uint8_t(0xF4)
#define BME280_CONFIG_ADDR					uint8_t(0xF5)
#define BME280_DATA_ADDR					uint8_t(0xF7)

/**\name API success code */
#define BME280_OK					int8_t(0)

/**\name API error codes */
#define BME280_E_NULL_PTR			int8_t(-1)
#define BME280_E_DEV_NOT_FOUND		int8_t(-2)
#define BME280_E_INVALID_LEN		int8_t(-3)
#define BME280_E_COMM_FAIL			int8_t(-4)
#define BME280_E_SLEEP_MODE_FAIL	int8_t(-5)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO      int8_t(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN	uint8_t(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN		uint8_t(7)
#define BME280_P_T_H_DATA_LEN				uint8_t(8)

/**\name Sensor power modes */
#define	BME280_SLEEP_MODE		uint8_t(0x00)
#define	BME280_FORCED_MODE		uint8_t(0x01)
#define	BME280_NORMAL_MODE		uint8_t(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
				((reg_data & ~(bitname##_MSK)) | \
				(data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)  ((reg_data & (bitname##_MSK)) >> \
							(bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK	uint8_t(0x03)
#define BME280_SENSOR_MODE_POS	uint8_t(0x00)

#define BME280_CTRL_HUM_MSK		uint8_t(0x07)
#define BME280_CTRL_HUM_POS		uint8_t(0x00)

#define BME280_CTRL_PRESS_MSK	uint8_t(0x1C)
#define BME280_CTRL_PRESS_POS	uint8_t(0x02)

#define BME280_CTRL_TEMP_MSK	uint8_t(0xE0)
#define BME280_CTRL_TEMP_POS	uint8_t(0x05)

#define BME280_FILTER_MSK		uint8_t(0x1C)
#define BME280_FILTER_POS		uint8_t(0x02)

#define BME280_STANDBY_MSK		uint8_t(0xE0)
#define BME280_STANDBY_POS		uint8_t(0x05)

/**\name Sensor component selection macros
   These values are internal for API implementation. Don't relate this to
   data sheet.*/
#define BME280_PRESS		uint8_t(1)
#define BME280_TEMP			uint8_t(1 << 1)
#define BME280_HUM			uint8_t(1 << 2)
#define BME280_ALL			uint8_t(0x07)

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL		uint8_t(1)
#define BME280_OSR_TEMP_SEL			uint8_t(1 << 1)
#define BME280_OSR_HUM_SEL			uint8_t(1 << 2)
#define BME280_FILTER_SEL			uint8_t(1 << 3)
#define BME280_STANDBY_SEL			uint8_t(1 << 4)
#define BME280_ALL_SETTINGS_SEL		uint8_t(0x1F)

#include <chrono>
#include <thread>
#include <iostream>

class IO
{
public:
    virtual int8_t read(uint8_t reg_addr,uint8_t *data, uint16_t len)=0;
    virtual int8_t write(uint8_t reg_addr,uint8_t *data, uint16_t len)=0;
    virtual ~IO(){};
    std::string& getInterfaceName()
    {
        return m_interfaceName;
    }
protected:
    std::string m_interfaceName{""};
};

class I2C : public IO
{
public:
    I2C(const std::string& path,const std::string& adress):m_path(path)
    {
        m_adress=static_cast<uint8_t>(std::stoi(adress,0,16));
        m_interfaceName="I2C";
    };
    virtual int8_t read(uint8_t reg_addr,uint8_t *data, uint16_t len)
    {
        ::write(m_fd, &reg_addr,1);
        ::read(m_fd, data, len);
        return 0; 
    }
    virtual int8_t write(uint8_t reg_addr,uint8_t *data, uint16_t len)
    {
        int8_t *buf;
        buf =static_cast<int8_t*>(malloc(len +1));
        buf[0] = reg_addr;
        memcpy(buf +1, data, len);
        ::write(m_fd, buf, len +1);
        free(buf);
        return 0;
    }
    int8_t connect()
    {
        if((m_fd = open(m_path.c_str(),O_RDWR)) < 0) 
        {
            printf("Failed to open the i2c bus %s",m_path.c_str());
            exit(1);
        }
        if(ioctl(m_fd,I2C_SLAVE,m_adress) < 0) 
        {
            printf("Failed to acquire bus access and/or talk to slave.\n");
            exit(1);
        }
    }
private:
    int m_fd;
    std::string m_path{""};
    uint8_t m_adress;
};
#endif /* BME280_DEFS_H_ */
/** @}*/
/** @}*/
