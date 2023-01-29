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
#define MPU6500_XG_OFFSET_H         0x13
#define MPU6500_XG_OFFSET_L         0x14
#define MPU6500_YG_OFFSET_H         0x15
#define MPU6500_YG_OFFSET_L         0x16
#define MPU6500_ZG_OFFSET_H         0x17
#define MPU6500_ZG_OFFSET_L         0x18
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
#define MPU6500_XA_OFFSET_H         0x77
#define MPU6500_XA_OFFSET_L         0x78
#define MPU6500_YA_OFFSET_H         0x7A
#define MPU6500_YA_OFFSET_L         0x7B
#define MPU6500_ZA_OFFSET_H         0x7D
#define MPU6500_ZA_OFFSET_L         0x7E
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
} ak8963_raw_mag_value_t;

typedef struct {
    float asa_x;
    float asa_y;
    float asa_z;
} ak8963_asa_value_t;

typedef struct {
    float asa_adjust_x;
    float asa_adjust_y;
    float asa_adjust_z;
} ak8963_asa_adjust_value_t;

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
} ak8963_mag_value_t;

typedef struct {
    float roll;
    float pitch;
} complimentary_angle_t;

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
    mpu6500_acce_fs_t ACCEL_FS;
    mpu6500_gyro_fs_t GYRO_FS;
    float acce_sensitivity;
    float gyro_sensitivity;
    mpu6500_acce_value_t acce_values;
    mpu6500_gyro_value_t gyro_values;
} mpu6500_dev_t;

typedef struct {
    i2c_bus_device_handle_t i2c_dev;
    uint8_t dev_addr;
    ak8963_asa_value_t asa_values;
    ak8963_asa_adjust_value_t asa_adjust;
    ak8963_mag_value_t mag_values;
} ak8963_dev_t;

typedef struct {
    mpu6500_dev_t *mpu6500;
    ak8963_dev_t  *ak8963;
    uint32_t counter;
    float dt;  /*!< delay time between twice measurement, dt should be small (ms level) */
    struct timeval *timer;
} mpu9250_dev_t;

typedef void *mpu9250_handle_t;
typedef void *mpu6500_handle_t;
typedef void *ak8963_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
mpu6500_handle_t mpu6500_create(uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of mpu6500
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_delete(mpu6500_dev_t *sensor);

/**
 * @brief Check the presence of MPU6500 on bus
 * 
 * @return 
 *      -ESP_OK device found 
 *      -ESP_FAIL device not found
*/
esp_err_t mpu6500_get_deviceid();

/**
 * @brief Read raw accelerometer measurements
 *
 * @param raw_acce_value raw accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_raw_acce(mpu6500_raw_acce_value_t *raw_acce_value);

/**
 * @brief Read raw gyroscope measurements
 *
 * @param raw_gyro_value raw gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_raw_gyro(mpu6500_raw_gyro_value_t *raw_gyro_value);

/**
 * @brief Get accelerometer sensitivity
 *
 * @param acce_sensitivity accelerometer sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_acce_sensitivity(void);

/**
 * @brief Get gyroscope sensitivity
 *
 * @param gyro_sensitivity gyroscope sensitivity
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_gyro_sensitivity(void);

/**
 * @brief Read accelerometer measurements
 *
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_acce(mpu6500_acce_value_t *acce_value);

/**
 * @brief Read gyroscope measurements
 *
 * @param acce_value gyroscope measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mpu6500_get_gyro( mpu6500_gyro_value_t *gyro_value);

/**
 * @brief Get accelerometer and gyro offsets and save in respective registers
 * 
 * @return
 *      -ESP_OK Success
 *      -ESP_FAIL Fail
*/
esp_err_t mpu6500_calibrate_acce_gyro(void);


/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
ak8963_handle_t ak8963_create(uint8_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor point to object handle of ak8963
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ak8963_delete(ak8963_dev_t *sensor);

/**
 * @brief Check the presence of AK8963 on bus
 * 
 * @return 
 *      -ESP_OK device found 
 *      -ESP_FAIL device not found
*/
esp_err_t ak8963_get_deviceid();

/**
 * @brief Read raw magnetometer measurements
 *
 * @param raw_gyro_value raw magnetometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ak8963_get_raw_mag(ak8963_raw_mag_value_t *raw_mag_value);

/**
 * @brief Read magnetometer measurements
 *
 * @param mag_value raw magnetometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ak8963_get_mag(ak8963_mag_value_t *mag_value);

/**
 * @brief Create and init the mpu9250 object
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
esp_err_t mpu9250_create();

/**
 * @brief Delete the mpu9250 object
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
esp_err_t mpu9250_delete();

/**
 * @brief acquire mpu6500 accelerometer result one time. Units: g
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu9250_acquire_acce();

/**
 * @brief acquire mpu6500 gyroscope result one time. Units:dps
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu9250_acquire_gyro();

/**
 * @brief acquire ak8963 magnetometer result one time. Units:dps
 * 
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t imu_mpu9250_acquire_mag();


#ifdef __cplusplus
}
#endif

#endif
