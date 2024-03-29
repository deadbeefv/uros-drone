// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef __MPU9250_H_
#define __MPU9250_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c_bus.h"

#define MPU6500_I2C_ADDRESS         0x68    /*!< slave address for MPU6500 sensor */
#define AK8963_I2C_ADDRESS          0x0C    /*!< slave address for AK8963 sensor */

/* MPU6500 register */
#define MPU6500_SELF_TEST_X         0x0D
#define MPU6500_SELF_TEST_Y         0x0E
#define MPU6500_SELF_TEST_Z         0x0F
#define MPU6500_SELF_TEST_A         0x10
#define MPU6500_SMPLRT_DIV          0x19
#define MPU6500_CONFIG              0x1A
#define MPU6500_GYRO_CONFIG         0x1B
#define MPU6500_ACCEL_CONFIG        0x1C
#define MPU6500_FIFO_EN             0x23
#define MPU6500_I2C_MST_CTRL        0x24
#define MPU6500_I2C_SLV0_ADDR       0x25
#define MPU6500_I2C_SLV0_REG        0x26
#define MPU6500_I2C_SLV0_CTRL       0x27
#define MPU6500_I2C_SLV1_ADDR       0x28
#define MPU6500_I2C_SLV1_REG        0x29
#define MPU6500_I2C_SLV1_CTRL       0x2A
#define MPU6500_I2C_SLV2_ADDR       0x2B
#define MPU6500_I2C_SLV2_REG        0x2C
#define MPU6500_I2C_SLV2_CTRL       0x2D
#define MPU6500_I2C_SLV3_ADDR       0x2E
#define MPU6500_I2C_SLV3_REG        0x2F
#define MPU6500_I2C_SLV3_CTRL       0x30
#define MPU6500_I2C_SLV4_ADDR       0x31
#define MPU6500_I2C_SLV4_REG        0x32
#define MPU6500_I2C_SLV4_DO         0x33
#define MPU6500_I2C_SLV4_CTRL       0x34
#define MPU6500_I2C_SLV4_DI         0x35
#define MPU6500_I2C_MST_STATUS      0x36
#define MPU6500_INT_PIN_CFG         0x37
#define MPU6500_INT_ENABLE          0x38
#define MPU6500_DMP_INT_STATUS      0x39
#define MPU6500_INT_STATUS          0x3A
#define MPU6500_ACCEL_XOUT_H        0x3B
#define MPU6500_ACCEL_XOUT_L        0x3C
#define MPU6500_ACCEL_YOUT_H        0x3D
#define MPU6500_ACCEL_YOUT_L        0x3E
#define MPU6500_ACCEL_ZOUT_H        0x3F
#define MPU6500_ACCEL_ZOUT_L        0x40
#define MPU6500_TEMP_OUT_H          0x41
#define MPU6500_TEMP_OUT_L          0x42
#define MPU6500_GYRO_XOUT_H         0x43
#define MPU6500_GYRO_XOUT_L         0x44
#define MPU6500_GYRO_YOUT_H         0x45
#define MPU6500_GYRO_YOUT_L         0x46
#define MPU6500_GYRO_ZOUT_H         0x47
#define MPU6500_GYRO_ZOUT_L         0x48
#define MPU6500_EXT_SENS_DATA_00    0x49
#define MPU6500_EXT_SENS_DATA_01    0x4A
#define MPU6500_EXT_SENS_DATA_02    0x4B
#define MPU6500_EXT_SENS_DATA_03    0x4C
#define MPU6500_EXT_SENS_DATA_04    0x4D
#define MPU6500_EXT_SENS_DATA_05    0x4E
#define MPU6500_EXT_SENS_DATA_06    0x4F
#define MPU6500_EXT_SENS_DATA_07    0x50
#define MPU6500_EXT_SENS_DATA_08    0x51
#define MPU6500_EXT_SENS_DATA_09    0x52
#define MPU6500_EXT_SENS_DATA_10    0x53
#define MPU6500_EXT_SENS_DATA_11    0x54
#define MPU6500_EXT_SENS_DATA_12    0x55
#define MPU6500_EXT_SENS_DATA_13    0x56
#define MPU6500_EXT_SENS_DATA_14    0x57
#define MPU6500_EXT_SENS_DATA_15    0x58
#define MPU6500_EXT_SENS_DATA_16    0x59
#define MPU6500_EXT_SENS_DATA_17    0x5A
#define MPU6500_EXT_SENS_DATA_18    0x5B
#define MPU6500_EXT_SENS_DATA_19    0x5C
#define MPU6500_EXT_SENS_DATA_20    0x5D
#define MPU6500_EXT_SENS_DATA_21    0x5E
#define MPU6500_EXT_SENS_DATA_22    0x5F
#define MPU6500_EXT_SENS_DATA_23    0x60
#define MPU6500_I2C_SLV0_DO         0x63
#define MPU6500_I2C_SLV1_DO         0x64
#define MPU6500_I2C_SLV2_DO         0x65
#define MPU6500_I2C_SLV3_DO         0x66
#define MPU6500_I2C_MST_DELAY_CTRL  0x67
#define MPU6500_SIGNAL_PATH_RESET   0x68
#define MPU6500_USER_CTRL           0x6A
#define MPU6500_PWR_MGMT_1          0x6B
#define MPU6500_PWR_MGMT_2          0x6C
#define MPU6500_FIFO_COUNTH         0x72
#define MPU6500_FIFO_COUNTL         0x73
#define MPU6500_FIFO_R_W            0x74
#define MPU6500_WHO_AM_I            0x75
#define MPU6500_I2C_SLV0_EN         0x80

/* AK8963 registers */
#define AK8963_I2C_ADDR     0x0C
#define AK8963_ST1          0x02
#define AK8963_ST2          0x09
#define AK8963_HXL          0x03
#define AK8963_CNTL1        0x0A
#define AK8963_CNTL2        0x0B
#define AK8963_PWR_DOWN     0x00
#define AK8963_HOFL_MASK    0x08
#define AK8963_CNT_MEAS1    0x12
#define AK8963_CNT_MEAS2    0x16
#define AK8963_FUSE_ROM     0x0F
#define AK8963_CNTL2        0x0B
#define AK8963_RESET        0x01
#define AK8963_ASA          0x10
#define AK8963_WHO_AM_I     0x00

typedef enum {
    ACCE_FS_2G  = 0,     /*!< Accelerometer full scale range is +/- 2g */
    ACCE_FS_4G  = 1,     /*!< Accelerometer full scale range is +/- 4g */
    ACCE_FS_8G  = 2,     /*!< Accelerometer full scale range is +/- 8g */
    ACCE_FS_16G = 3,     /*!< Accelerometer full scale range is +/- 16g */
} mpu6500_acce_fs_t;

typedef enum {
    GYRO_FS_250DPS  = 0,     /*!< Gyroscope full scale range is +/- 250 degree per sencond */
    GYRO_FS_500DPS  = 1,     /*!< Gyroscope full scale range is +/- 500 degree per sencond */
    GYRO_FS_1000DPS = 2,     /*!< Gyroscope full scale range is +/- 1000 degree per sencond */
    GYRO_FS_2000DPS = 3,     /*!< Gyroscope full scale range is +/- 2000 degree per sencond */
} mpu6500_gyro_fs_t;

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} mpu6500_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} mpu6500_raw_gyro_value_t;

typedef struct {
    int16_t raw_mag_x;
    int16_t raw_mag_y;
    int16_t raw_mag_z;
} mpu6500_raw_mag_value_t;

typedef struct {
    int8_t asa_x;
    int8_t asa_y;
    int8_t asa_z;
} ak8963_asa_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} mpu6500_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6500_gyro_value_t;

typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
} mpu6500_mag_value_t;

typedef struct {
    float roll;
    float pitch;
} complimentary_angle_t;

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between twice measurement, dt should be small (ms level) */
    struct timeval *timer;
} mpu6500_dev_t;

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
} ak8963_dev_t;


typedef void *mpu6500_handle_t;
typedef void *ak8963_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
mpu6500_handle_t mpu6500_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
ak8963_handle_t ak8963_create(i2c_bus_handle_t bus, uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of mpu6500
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_delete(mpu6500_handle_t *sensor);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of ak8963
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ak8963_delete(ak8963_handle_t *sensor);

/**
 * @brief Get device identification of MPU6500
 *
 * @param sensor object handle of mpu6500
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_deviceid(mpu6500_handle_t sensor, uint8_t *deviceid);

/**
 * @brief Get device identification of AK8963
 *
 * @param sensor object handle of mpu6500
 * @param deviceid a pointer of device ID
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ak8963_get_deviceid(mpu6500_handle_t sensor, uint8_t *deviceid);

/**
 * @brief Wake up MPU6500
 *
 * @param sensor object handle of mpu6500
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_wake_up(mpu6500_handle_t sensor);

/**
 * @brief Wake up AK8963
 *
 * @param sensor object handle of ak8963
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ak8963_wake_up(mpu6500_handle_t sensor);

/**
 * @brief Enter sleep mode
 *
 * @param sensor object handle of mpu6500
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_sleep(mpu6500_handle_t sensor);

/**
 * @brief Set accelerometer full scale range
 *
 * @param sensor object handle of mpu6500
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_set_acce_fs(mpu6500_handle_t sensor, mpu6500_acce_fs_t acce_fs);

/**
 * @brief Set gyroscope full scale range
 *
 * @param sensor object handle of mpu6500
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_set_gyro_fs(mpu6500_handle_t sensor, mpu6500_gyro_fs_t gyro_fs);

/**
 * @brief Get accelerometer full scale range
 *
 * @param sensor object handle of mpu6500
 * @param acce_fs accelerometer full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_acce_fs(mpu6500_handle_t sensor, mpu6500_acce_fs_t *acce_fs);

/**
 * @brief Get gyroscope full scale range
 *
 * @param sensor object handle of mpu6500
 * @param gyro_fs gyroscope full scale range
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_gyro_fs(mpu6500_handle_t sensor, mpu6500_gyro_fs_t *gyro_fs);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param sensor object handle of mpu6500
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_acce_sensitivity(mpu6500_handle_t sensor, float *acce_sensitivity);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param sensor object handle of mpu6500
 * @param gyro_sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_gyro_sensitivity(mpu6500_handle_t sensor, float *gyro_sensitivity);

/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of mpu6500
 * @param acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_raw_acce(mpu6500_handle_t sensor, mpu6500_raw_acce_value_t *raw_acce_value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param sensor object handle of mpu6500
 * @param gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_raw_gyro(mpu6500_handle_t sensor, mpu6500_raw_gyro_value_t *raw_gyro_value);

/**
 * @brief Read raw magnetometer measurements
 *
 * @param sensor object handle of ak8963
 * @param gyro_value raw magnetometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_raw_mag(mpu6500_handle_t sensor, mpu6500_raw_mag_value_t *raw_mag_value);

/**
 * @brief Read accelerometer measurements
 *
 * @param sensor object handle of mpu6500
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_acce(mpu6500_handle_t sensor, mpu6500_acce_value_t *acce_value);

/**
 * @brief Read gyroscope measurements
 *
 * @param sensor object handle of mpu6500
 * @param acce_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_gyro(mpu6500_handle_t sensor, mpu6500_gyro_value_t *gyro_value);

/**
 * @brief use complimentory filter to caculate roll and pitch
 *
 * @param acce_value accelerometer measurements
 * @param gyro_value gyroscope measurements
 * @param complimentary_angle complimentary angle
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_complimentory_filter(mpu6500_handle_t sensor, mpu6500_acce_value_t *acce_value,
        mpu6500_gyro_value_t *gyro_value, complimentary_angle_t *complimentary_angle);

/***implements of imu hal interface****/

/**
 * @brief initialize mpu6500 with default configurations
 * 
 * @param i2c_bus i2c bus handle the sensor will attached to
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu6500_init(i2c_bus_handle_t i2c_bus);

esp_err_t mag_ak8963_init(mpu6500_handle_t sensor);

esp_err_t ak8963_read_registers(mpu6500_handle_t sensor, uint8_t sub_address, uint8_t count, uint8_t *dest);

esp_err_t ak8963_write_register(mpu6500_handle_t sensor, uint8_t sub_address, uint8_t data);

esp_err_t ak8963_get_mode(mpu6500_handle_t sensor, uint8_t *mode);

esp_err_t ak8963_set_mode(mpu6500_handle_t sensor, uint8_t mode);

esp_err_t ak8963_get_asa(mpu6500_handle_t sensor);

esp_err_t ak8963_reset(mpu6500_handle_t sensor);

esp_err_t mpu6500_get_mag(mpu6500_handle_t sensor, mpu6500_mag_value_t *mag_value);


/**
 * @brief de-initialize mpu6500
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */

esp_err_t imu_mpu6500_deinit(void);

/**
 * @brief test if mpu6500 is active
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu6500_test(void);

/**
 * @brief acquire mpu6500 accelerometer result one time.
 * 
 * @param acce_x result data (unit:g)
 * @param acce_y result data (unit:g)
 * @param acce_z result data (unit:g)
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu6500_acquire_acce(mpu6500_acce_value_t *acce);

/**
 * @brief acquire mpu6500 gyroscope result one time.
 * 
 * @param gyro_x result data (unit:dps)
 * @param gyro_y result data (unit:dps)
 * @param gyro_z result data (unit:dps)
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu6500_acquire_gyro(mpu6500_gyro_value_t *gyro);

esp_err_t imu_mpu6500_acquire_mag(mpu6500_mag_value_t *mag);

/**
 * @brief set mpu6500 to sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu6500_sleep(void);

/**
 * @brief wakeup mpu6500 from sleep mode.
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu6500_wakeup(void);


#ifdef __cplusplus
}
#endif

#endif

