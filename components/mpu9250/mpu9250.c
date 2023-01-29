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
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "driver/i2c.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "bus_config.h"
#include "mpu9250.h"


#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define ALPHA 0.99             /*!< Weight for gyroscope */
#define RAD_TO_DEG 57.27272727 /*!< Radians to degrees */

static const char *MPU6500_TAG = "MPU6500";
mpu6500_handle_t mpu6500 = NULL;
ak8963_handle_t ak8963 = NULL;
static bool is_init = false;

mpu6500_acce_value_t acce = {0, 0, 0};
mpu6500_gyro_value_t gyro = {0, 0, 0};
mpu6500_mag_value_t mag = {0, 0, 0};
ak8963_asa_value_t asa_values = {0, 0, 0};

mpu6500_handle_t mpu6500_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    if (bus == NULL) {
        return NULL;
    }

    mpu6500_dev_t *sens = (mpu6500_dev_t *) calloc(1, sizeof(mpu6500_dev_t));
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = dev_addr;
    sens->counter = 0;
    sens->dt = 0;
    sens->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6500_handle_t) sens;
}

ak8963_handle_t ak8963_create(i2c_bus_handle_t bus, uint8_t dev_addr)
{
    if (bus == NULL){
        return NULL;
    }

    mpu6500_dev_t *sens = (mpu6500_dev_t *) bus;
    sens->i2c_dev = i2c_bus_device_create(bus, dev_addr, i2c_bus_get_current_clk_speed(bus));
    if (sens->i2c_dev == NULL) {
        free(sens);
        return NULL;
    }
    sens->dev_addr = dev_addr;
    return (ak8963_handle_t) sens;
}

esp_err_t mpu6500_delete(mpu6500_handle_t *sensor)
{
    if (*sensor == NULL) {
        return ESP_OK;
    }

    mpu6500_dev_t *sens = (mpu6500_dev_t *)(*sensor);
    i2c_bus_device_delete(&sens->i2c_dev);
    free(sens->timer);
    free(sens);
    *sensor = NULL;
    return ESP_OK;
}

esp_err_t ak8963_delete(ak8963_handle_t *sensor)
{
    if (*sensor == NULL) {
        return ESP_OK;
    }

    ak8963_dev_t *sens = (ak8963_dev_t *)(*sensor);
    i2c_bus_device_delete(&sens->i2c_dev);
    free(sens);
    *sensor = NULL;
    return ESP_OK;
}

esp_err_t mpu6500_get_deviceid(mpu6500_handle_t sensor, uint8_t *deviceid)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_WHO_AM_I, deviceid);
    return ret;
}

esp_err_t ak8963_get_deviceid(mpu6500_handle_t sensor, uint8_t *deviceid)
{
    uint8_t count = 1;
    esp_err_t ret = ak8963_read_registers(sensor, AK8963_WHO_AM_I, count, deviceid);
    return ret;
}

esp_err_t mpu6500_wake_up(mpu6500_handle_t sensor)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_PWR_MGMT_1, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }

    tmp &= (~BIT6);
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_PWR_MGMT_1, tmp);
    return ret;
}

esp_err_t ak8963_wake_up(mpu6500_handle_t sensor)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp = 0xFF;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_INT_PIN_CFG, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }
    return ret;
}

esp_err_t ak8963_reset(mpu6500_handle_t sensor)
{
    return ak8963_write_register(sensor, AK8963_CNTL2, AK8963_RESET);
}

esp_err_t mpu6500_sleep(mpu6500_handle_t sensor)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_PWR_MGMT_1, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }

    tmp |= BIT6;
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_PWR_MGMT_1, tmp);
    return ret;
}

esp_err_t mpu6500_set_acce_fs(mpu6500_handle_t sensor, mpu6500_acce_fs_t acce_fs)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }

    tmp &= (~BIT3);
    tmp &= (~BIT4);
    tmp |= (acce_fs << 3);
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, tmp);
    return ret;
}

esp_err_t mpu6500_set_gyro_fs(mpu6500_handle_t sensor, mpu6500_gyro_fs_t gyro_fs)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, &tmp);

    if (ret != ESP_OK) {
        return ret;
    }

    tmp &= (~BIT3);
    tmp &= (~BIT4);
    tmp |= (gyro_fs << 3);
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, tmp);
    return ret;
}

esp_err_t mpu6500_get_acce_fs(mpu6500_handle_t sensor, mpu6500_acce_fs_t *acce_fs)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, &tmp);
    tmp = (tmp >> 3) & 0x03;
    *acce_fs = tmp;
    return ret;
}

esp_err_t mpu6500_get_gyro_fs(mpu6500_handle_t sensor, mpu6500_gyro_fs_t *gyro_fs)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t tmp;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, &tmp);
    tmp = (tmp >> 3) & 0x03;
    *gyro_fs = tmp;
    return ret;
}

esp_err_t mpu6500_get_acce_sensitivity(mpu6500_handle_t sensor, float *acce_sensitivity)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t acce_fs;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, &acce_fs);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
        case ACCE_FS_2G:
            *acce_sensitivity = 16384;
            break;
        case ACCE_FS_4G:
            *acce_sensitivity = 8192;
            break;
        case ACCE_FS_8G:
            *acce_sensitivity = 4096;
            break;
        case ACCE_FS_16G:
            *acce_sensitivity = 2048;
            break;
        default:
            break;
    }
    return ret;
}

esp_err_t mpu6500_get_gyro_sensitivity(mpu6500_handle_t sensor, float *gyro_sensitivity)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, &gyro_fs);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
        case GYRO_FS_250DPS:
            *gyro_sensitivity = 131;
            break;
        case GYRO_FS_500DPS:
            *gyro_sensitivity = 65.5;
            break;
        case GYRO_FS_1000DPS:
            *gyro_sensitivity = 32.8;
            break;
        case GYRO_FS_2000DPS:
            *gyro_sensitivity = 16.4;
            break;
        default:
            break;
    }
    return ret;
}

esp_err_t mpu6500_get_raw_acce(mpu6500_handle_t sensor, mpu6500_raw_acce_value_t *raw_acce_value)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    uint8_t data_rd[6] = {0};
    esp_err_t ret = i2c_bus_read_bytes(sens->i2c_dev, MPU6500_ACCEL_XOUT_H, 6, data_rd);
    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6500_get_raw_gyro(mpu6500_handle_t sensor, mpu6500_raw_gyro_value_t *raw_gyro_value)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    uint8_t data_rd[6] = {0};
    esp_err_t ret = i2c_bus_read_bytes(sens->i2c_dev, MPU6500_GYRO_XOUT_H, 6, data_rd);
    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6500_get_raw_mag(mpu6500_handle_t sensor, mpu6500_raw_mag_value_t *raw_mag_value)
{
    uint8_t data_rd[7] = {0};
    uint8_t drdy;

    esp_err_t ret = ak8963_read_registers(sensor, AK8963_ST1, 1, &drdy);

    if (ret != ESP_OK){
        return ret;
    }

    if (drdy & 0x01){
        ret = ak8963_read_registers(sensor, AK8963_HXL, 7, data_rd);
        if (ret == ESP_OK){
            raw_mag_value->raw_mag_x = (int16_t)((data_rd[1] << 8) | (data_rd[0]));
            raw_mag_value->raw_mag_y = (int16_t)((data_rd[3] << 8) | (data_rd[2]));
            raw_mag_value->raw_mag_x = (int16_t)((data_rd[5] << 8) | (data_rd[4]));
            printf("RawMag_x: %d\t\tRawMag_y: %d\t\tRawMag_z:%d\n", raw_mag_value->raw_mag_x, raw_mag_value->raw_mag_y, raw_mag_value->raw_mag_z);
        }
    }
    return ret;
}

esp_err_t mpu6500_get_acce(mpu6500_handle_t sensor, mpu6500_acce_value_t *acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6500_raw_acce_value_t raw_acce;
    ret = mpu6500_get_acce_sensitivity(sensor, &acce_sensitivity);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6500_get_raw_acce(sensor, &raw_acce);

    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6500_get_gyro(mpu6500_handle_t sensor, mpu6500_gyro_value_t *gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6500_raw_gyro_value_t raw_gyro;
    ret = mpu6500_get_gyro_sensitivity(sensor, &gyro_sensitivity);

    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6500_get_raw_gyro(sensor, &raw_gyro);

    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6500_get_mag(mpu6500_handle_t sensor, mpu6500_mag_value_t *mag_value)
{
    mpu6500_raw_mag_value_t raw_mag;
    esp_err_t ret = mpu6500_get_raw_mag(mpu6500, &raw_mag);
    if (ret != ESP_OK){
        return ret;
    }

    float scale_factor = 0.1499;

    mag_value->mag_x = raw_mag.raw_mag_x * scale_factor * (((asa_values.asa_x - 128)*0.5)/128.0 + 1);
    mag_value->mag_y = raw_mag.raw_mag_y * scale_factor * (((asa_values.asa_y - 128)*0.5)/128.0 + 1);
    mag_value->mag_z = raw_mag.raw_mag_z * scale_factor * (((asa_values.asa_z - 128)*0.5)/128.0 + 1);
    return ESP_OK;
}

esp_err_t mpu6500_complimentory_filter(mpu6500_handle_t sensor, mpu6500_acce_value_t *acce_value,
        mpu6500_gyro_value_t *gyro_value, complimentary_angle_t *complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    sens->counter++;

    if (sens->counter == 1) {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float)(dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);
    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;
    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);
    return ESP_OK;
}

/***sensors hal interface****/

esp_err_t imu_mpu6500_init(i2c_bus_handle_t i2c_bus)
{
    if (is_init || !i2c_bus) {
        return ESP_FAIL;
    }

    mpu6500 = mpu6500_create(i2c_bus, MPU6500_I2C_ADDRESS);

    if (!mpu6500) {
        return ESP_FAIL;
    }

    uint8_t mpu6500_deviceid;
    mpu6500_get_deviceid(mpu6500, &mpu6500_deviceid);
    ESP_LOGI(MPU6500_TAG, "mpu6500 device address is: 0x%02x\n", mpu6500_deviceid);
    esp_err_t ret = mpu6500_wake_up(mpu6500);
    ret = mpu6500_set_acce_fs(mpu6500, ACCE_FS_2G);
    ret = mpu6500_set_gyro_fs(mpu6500, GYRO_FS_500DPS);

    if (ret == ESP_OK) {
        is_init = true;
    }
    return ret;
}

esp_err_t mag_ak8963_init(mpu6500_handle_t sensor)
{
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;

    //enable I2C master mode
    esp_err_t ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_USER_CTRL,0x20);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    //reset device
    ak8963_reset(sensor);

    //get sensitivity adjustment values
    ak8963_get_asa(sensor);

    //set mode to continuous measurement 2
    ret = ak8963_write_register(sensor, AK8963_CNTL1, AK8963_CNT_MEAS2);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t ak8963_write_register(mpu6500_handle_t sensor, uint8_t sub_address, uint8_t data)
{
    //set slave0 to the ak8963 in write mode
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    //send the register of the slave address
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_REG, sub_address);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    //store the data to be written
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_DO, data);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    //enable I2C to the slave and set the mode
    uint8_t count = 1;
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLV0_EN | count);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    return ret;
}

esp_err_t ak8963_read_registers(mpu6500_handle_t sensor, uint8_t sub_address, uint8_t count, uint8_t *dest)
{
    //set slave 0 to read mode
    mpu6500_dev_t *sens = (mpu6500_dev_t *) sensor;
    esp_err_t ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_ADDR, AK8963_I2C_ADDR | MPU6500_I2C_SLV0_EN);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    //set the ak8926 register to be read
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_REG, sub_address);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    //enable I2C to slave 0 and request data
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_I2C_SLV0_CTRL, MPU6500_I2C_SLV0_EN | count);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    ret = i2c_bus_read_bytes(sens->i2c_dev, MPU6500_EXT_SENS_DATA_00, count, dest);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ak8963_get_mode(mpu6500_handle_t sensor, uint8_t *mode)
{
    esp_err_t ret = ak8963_read_registers(sensor, AK8963_CNTL1, 1, mode);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t ak8963_set_mode(mpu6500_handle_t sensor, uint8_t mode)
{
    // //power down the magnetometer
    // esp_err_t ret = ak8963_write_register(sensor, AK8963_CNTL1, AK8963_PWR_DOWN);
    // if (ret != ESP_OK){
    //     return ESP_FAIL;
    // }
    // set the mode
    esp_err_t ret = ak8963_write_register(sensor, AK8963_CNTL1, mode);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    //check if mode is correctly set
    // uint8_t new_mode;
    // ret = ak8963_read_registers(sensor, AK8963_CNTL1, 1, &new_mode);
    // if (ret == ESP_OK && mode == new_mode){
    //     return ESP_OK;
    // }
    return ESP_OK;
}

esp_err_t ak8963_get_asa(mpu6500_handle_t sensor)
{
    // power down the magnetometer
    esp_err_t ret = ak8963_write_register(sensor, AK8963_CNTL1, AK8963_PWR_DOWN);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    //enter fuse rom mode
    ret = ak8963_write_register(sensor, AK8963_CNTL1, AK8963_FUSE_ROM);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    //read sensitivity adjustment values from the sensor
    uint8_t sa[3] = {0};
    ret = ak8963_read_registers(sensor, AK8963_ASA, 3, sa);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    asa_values.asa_x = sa[0];
    asa_values.asa_y = sa[1];
    asa_values.asa_z = sa[2];

    //power down the magnetometer
    ret = ak8963_write_register(sensor, AK8963_CNTL1, AK8963_PWR_DOWN);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }

    //return the magnetometer back to continuous reading mode 2
    ret = ak8963_write_register(sensor, AK8963_CNTL1, AK8963_CNT_MEAS2);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t imu_mpu6500_deinit(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    esp_err_t ret = mpu6500_sleep(mpu6500);
    ret = mpu6500_delete(&mpu6500);

    if (ret == ESP_OK) {
        is_init = false;
    }

    return ret;
}

esp_err_t imu_mpu6500_sleep(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return mpu6500_sleep(mpu6500);
}

esp_err_t imu_mpu6500_wakeup(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return mpu6500_wake_up(mpu6500);
}

esp_err_t imu_mpu6500_test(void)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t imu_mpu6500_acquire_gyro(mpu6500_gyro_value_t *gyro_value)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    if (ESP_OK == mpu6500_get_gyro(mpu6500, &gyro)) {
        gyro_value->gyro_x = gyro.gyro_x;
        gyro_value->gyro_y = gyro.gyro_y;
        gyro_value->gyro_z = gyro.gyro_z;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t imu_mpu6500_acquire_acce(mpu6500_acce_value_t *acce_value)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    if (ESP_OK == mpu6500_get_acce(mpu6500, &acce)) {
        acce_value->acce_x = acce.acce_x;
        acce_value->acce_y = acce.acce_y;
        acce_value->acce_z = acce.acce_z;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t imu_mpu6500_acquire_mag(mpu6500_mag_value_t *mag_value)
{
    if (!is_init) {
        return ESP_FAIL;
    }

    if (ESP_OK == mpu6500_get_mag(mpu6500, &mag)) {
        mag_value->mag_x = mag.mag_x;
        mag_value->mag_y = mag.mag_y;
        mag_value->mag_z = mag.mag_z;
        return ESP_OK;
    }
    return ESP_FAIL;
}