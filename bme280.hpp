#ifndef BME280_H_
#define BME280_H_
#include <cstdint>
#include "IO.hpp"
#include "settings.hpp"
#include "data.hpp"
#include <iostream>

/********************************************************/
/*! @name		Common macros		        */
/********************************************************/

#ifndef BME280_FLOAT_ENABLE
/* #define BME280_FLOAT_ENABLE */
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
#define BME280_64BIT_ENABLE
#endif
#endif

/**\name BME280 chip identifier */
static constexpr uint8_t CHIP_ID{0x60};

/**\name Register Address */
static constexpr uint8_t CHIP_ID_ADDR{0xD0};
static constexpr uint8_t RESET_ADDR{0xE0};
static constexpr uint8_t TEMP_PRESS_CALIB_DATA_ADDR{0x88};
static constexpr uint8_t HUMIDITY_CALIB_DATA_ADDR{0xE1};
static constexpr uint8_t PWR_CTRL_ADDR{0xF4};
static constexpr uint8_t CTRL_HUM_ADDR{0xF2};
static constexpr uint8_t CTRL_MEAS_ADDR{0xF4};
static constexpr uint8_t CONFIG_ADDR{0xF5};
static constexpr uint8_t DATA_ADDR{0xF7};


/**\name Macros related to size */
static constexpr uint8_t TEMP_PRESS_CALIB_DATA_LEN{26};
static constexpr uint8_t HUMIDITY_CALIB_DATA_LEN{7};
static constexpr uint8_t P_T_H_DATA_LEN{8};

/**\name Sensor power modes */
static constexpr	uint8_t SLEEP_MODE{0x00};
static constexpr	uint8_t FORCED_MODE{0x01};
static constexpr	uint8_t NORMAL_MODE{0x03};

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
static constexpr uint8_t SENSOR_MODE_MSK{0x03};
static constexpr uint8_t SENSOR_MODE_POS{0x00};

static constexpr uint8_t CTRL_HUM_MSK{0x07};
static constexpr uint8_t CTRL_HUM_POS{0x00};

static constexpr uint8_t CTRL_PRESS_MSK{0x1C};
static constexpr uint8_t CTRL_PRESS_POS{0x02};

static constexpr uint8_t CTRL_TEMP_MSK{0xE0};
static constexpr uint8_t CTRL_TEMP_POS{0x05};

static constexpr uint8_t FILTER_MSK{0x1C};
static constexpr uint8_t FILTER_POS{0x02};

static constexpr uint8_t STANDBY_MSK{0xE0};
static constexpr uint8_t STANDBY_POS{0x05};

/**\name Sensor component selection macros
   These values are internal for API implementation. Don't relate this to
   data sheet.*/
static constexpr uint8_t PRESS{1};
static constexpr uint8_t TEMP{1 << 1};
static constexpr uint8_t HUM{1 << 2};
static constexpr uint8_t ALL{0x07};

/**\name Settings selection macros */
static constexpr uint8_t OSR_PRESS_SEL{1};
static constexpr uint8_t OSR_TEMP_SEL{1 << 1};
static constexpr uint8_t OSR_HUM_SEL{1 << 2};
static constexpr uint8_t FILTER_SEL{1 << 3};
static constexpr uint8_t STANDBY_SEL{1 << 4};
static constexpr uint8_t ALL_SETTINGS_SEL{0x1F};

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

private:
    IO* m_IO{nullptr};
    /*! Sensor settings */
	settings m_settings;
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
    
    /**\name API success code */
    static constexpr int8_t OK{0};

    /**\name API error codes */
    static constexpr int8_t E_NULL_PTR{-1};
    static constexpr int8_t E_DEV_NOT_FOUND{-2};
    static constexpr int8_t E_INVALID_LEN{-3};
    static constexpr int8_t E_COMM_FAIL{-4};
    static constexpr int8_t E_SLEEP_MODE_FAIL{-5};
    
    /**\name API warning codes */
    static constexpr int8_t W_INVALID_OSR_MACRO{1};
    
    /**\name Internal macros */
    /* To identify osr settings selected by user */
    static constexpr uint8_t OVERSAMPLING_SETTINGS{0x07};
    /* To identify filter and standby settings selected by user */
    static constexpr uint8_t FILTER_STANDBY_SETTINGS{0x18};
    
};
#endif
