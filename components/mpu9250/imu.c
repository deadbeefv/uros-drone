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
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "bus_config.h"
#include "imu.h"

static const char *TAG = "MPU9250";
mpu9250_dev_t mpu9250;
extern i2c_bus_handle_t sensor_bus;

//Temporary storage for IMU data
mpu6500_acce_value_t acce_values = {
    .acce_x = 0.0,
    .acce_y = 0.0,
    .acce_z = 0.0,
};
mpu6500_gyro_value_t gyro_values = {
    .gyro_x = 0.0,
    .gyro_y = 0.0,
    .gyro_z = 0.0,
};
ak8963_mag_value_t mag_values = {
    .mag_x = 0.0,
    .mag_y = 0.0,
    .mag_z = 0.0,
};

//Accel/Gyro readings from MPU6500
mpu6500_handle_t mpu6500_create(uint8_t dev_addr)
{
    if (!is_i2c_bus_initialized(SENSOR_BUS)){
        return NULL;
    }

    mpu6500_dev_t *sens = (mpu6500_dev_t *) calloc(1, sizeof(mpu6500_dev_t));
    sens->i2c_dev = i2c_bus_device_create(sensor_bus, dev_addr, i2c_bus_get_current_clk_speed(sensor_bus));
    if (sens->i2c_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create memory for mpu6500 sensor struct");
        free(sens);
        return NULL;
    }
    //wake up mpu6500
    esp_err_t ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to wakeup mpu6500 acce/gyro");
        mpu6500_delete(sens);
        return NULL;
    }
    sleep(0.1);
    ESP_LOGI(TAG, "Successfully woke up mpu6500");
    //get stable time source
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to set stable time on mpu6500 acce/gyro");
        mpu6500_delete(sens);
        return NULL;
    }
    ESP_LOGI(TAG, "Successfully set stable time for mpu6500");

    //configure acce/gyro 
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_CONFIG, 0x03);
    ret += i2c_bus_write_byte(sens->i2c_dev, MPU6500_SMPLRT_DIV, 0x04);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to set acce/gyro bandwidth and sample rates");
        mpu6500_delete(sens);
        return NULL;
    }

    //set gyro full scale range
    uint8_t c;
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, &c);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to read gyro full scale range");
        mpu6500_delete(sens);
        return NULL;
    }
    //clear self-test bits
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, c& ~0xE0);
    ret += i2c_bus_write_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, c& ~0x18);
    ret += i2c_bus_write_byte(sens->i2c_dev, MPU6500_GYRO_CONFIG, c| GYRO_FS_250DPS<<3);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to set gyro full scale range");
        mpu6500_delete(sens);
        return NULL;
    }

    //set acce full scale range
    ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, &c);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to read accel full scale range");
        mpu6500_delete(sens);
        return NULL;
    }
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, c & ~0xE0);
    ret += i2c_bus_write_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, c& ~0x18);
    ret += i2c_bus_write_byte(sens->i2c_dev, MPU6500_ACCEL_CONFIG, c| ACCE_FS_2G<<3);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to set accel full scale range");
        mpu6500_delete(sens);
        return NULL;
    }
    ESP_LOGI(TAG, "Successfully configured Accel/Gyro");

    //configure interrupts and bypass enable
    ret = i2c_bus_write_byte(sens->i2c_dev, MPU6500_INT_PIN_CFG, 0x22);
    ret += i2c_bus_write_byte(sens->i2c_dev, MPU6500_INT_ENABLE, 0x01);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to configure interrupts and bypass enable");
        mpu6500_delete(sens);
        return NULL;
    }
    ESP_LOGI(TAG, "Successfully configured interrupts and bypass enable");
    sens->dev_addr = dev_addr;
    return (mpu6500_handle_t) sens;
}

esp_err_t mpu6500_delete(mpu6500_dev_t *sensor)
{
    if (sensor == NULL) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_bus_device_delete(&sensor->i2c_dev);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to delete mpu6500 on sensor bus");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Successfully deleted mpu6500 on sensor bus");
    free(sensor);
    return ESP_OK;
}

esp_err_t mpu6500_get_deviceid()
{
    if (!is_i2c_bus_initialized(SENSOR_BUS)){
        ESP_LOGE(TAG, "Uninitialized sensor bus");
        return ESP_FAIL;
    }

    if (mpu9250.mpu6500 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    mpu6500_dev_t *sens = mpu9250.mpu6500;
    uint8_t deviceid;
    esp_err_t ret = i2c_bus_read_byte(sens->i2c_dev, MPU6500_WHO_AM_I, &deviceid);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to get device ID from mpu6500");
        return ret;
    }
    if (deviceid != 0x71){
        ESP_LOGW(TAG, "Wrong MPUxxxx device found on bus. Actual response: 0x%x", deviceid);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Found MPU9250 device on bus. Response code: 0x%x", deviceid);
    return ESP_OK;
}

esp_err_t mpu6500_get_raw_acce(mpu6500_raw_acce_value_t *raw_acce_value)
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGW(TAG, "Cannot read from uninitialized sensor");
        return ESP_FAIL;
    }
    uint8_t data_rd[6] = {0};
    esp_err_t ret = i2c_bus_read_bytes(mpu9250.mpu6500->i2c_dev, MPU6500_ACCEL_XOUT_H, 6, data_rd);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to read raw Accelerometer data");
        return ESP_FAIL;
    }
    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6500_get_raw_gyro(mpu6500_raw_gyro_value_t *raw_gyro_value)
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGW(TAG, "Cannot read from uninitialized sensor");
        return ESP_FAIL;
    }
    uint8_t data_rd[6] = {0};
    esp_err_t ret = i2c_bus_read_bytes(mpu9250.mpu6500->i2c_dev, MPU6500_GYRO_XOUT_H, 6, data_rd);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to read raw Gyroscope data");
        return ESP_FAIL;
    }
    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6500_get_acce_sensitivity()
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    uint8_t acce_fs;
    esp_err_t ret = i2c_bus_read_byte(mpu9250.mpu6500->i2c_dev, MPU6500_ACCEL_CONFIG, &acce_fs);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to read accelerometer sensitivity");
        return ESP_FAIL;
    } 
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
        case ACCE_FS_2G:
            mpu9250.mpu6500->acce_sensitivity = 16384;
            break;
        case ACCE_FS_4G:
            mpu9250.mpu6500->acce_sensitivity  = 8192;
            break;
        case ACCE_FS_8G:
            mpu9250.mpu6500->acce_sensitivity  = 4096;
            break;
        case ACCE_FS_16G:
            mpu9250.mpu6500->acce_sensitivity  = 2048;
            break;
    }
    return ret;
}

esp_err_t mpu6500_get_gyro_sensitivity()
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    uint8_t gyro_fs;
    esp_err_t ret = i2c_bus_read_byte(mpu9250.mpu6500->i2c_dev, MPU6500_ACCEL_CONFIG, &gyro_fs);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to read gyroscope sensitivity");
        return ESP_FAIL;
    } 
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
        case GYRO_FS_250DPS:
            mpu9250.mpu6500->gyro_sensitivity = 131;
            break;
        case GYRO_FS_500DPS:
            mpu9250.mpu6500->gyro_sensitivity = 65.5;
            break;
        case GYRO_FS_1000DPS:
            mpu9250.mpu6500->gyro_sensitivity = 32.8;
            break;
        case GYRO_FS_2000DPS:
            mpu9250.mpu6500->gyro_sensitivity = 16.4;
            break;
    }
    return ret;
}

esp_err_t mpu6500_get_gyro(mpu6500_gyro_value_t *gyro_value)
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    
    mpu6500_raw_gyro_value_t raw_gyro;
    esp_err_t ret = mpu6500_get_raw_gyro(&raw_gyro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw gyro values");;
        return ret;
    }
    float gyro_sensitivity = mpu9250.mpu6500->gyro_sensitivity;
    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6500_get_acce(mpu6500_acce_value_t *acce_value)
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    mpu6500_raw_acce_value_t raw_acce;
    esp_err_t ret = mpu6500_get_raw_acce(&raw_acce);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Faiied to read raw accelerometer values");
        return ret;
    }
    float acce_sensitivity = mpu9250.mpu6500->acce_sensitivity;
    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6500_calibrate_acce_gyro(void)
{
    if (mpu9250.mpu6500 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    
    uint8_t data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    uint16_t i, pkt_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0};
    int32_t acce_bias[3] = {0, 0, 0};

    //Reset acce/gyro
    esp_err_t ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to power down mpu6500");
        return ret;
    }
    sleep(0.1);

    //get stable time source by setting clock source to PLL with  x-axis gyro
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_PWR_MGMT_1, 0x01);
    ret += i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to get stable time source");
        return ret;
    }
    sleep(0.2);

    // Configure accel/gyro for bias calculation
    // Disable Interrupts
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_INT_PIN_CFG, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to disable interrupts");
        return ret;
    }

    //Disable FIFO
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_FIFO_EN, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to disable FIFO");
        return ret;
    }

    //Turn on internal clock source
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to disable internal clock source");
        return ret;
    }

    //Disable I2C master
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_I2C_MST_CTRL, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to disable master control");
        return ret;
    }

    //Disable FIFO and I2C master mode
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_USER_CTRL, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to disable FIFO and I2C master mode");
        return ret;
    }

    //Reset FIFO and DMP
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_USER_CTRL, 0x0C);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to reset FIFO and DMP");
        return ret;
    }

    //Configure gyro and acce for bias calculation
    //Set LPF to 188Hz
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_CONFIG, 0x01);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to configure low pass filter");
        return ret;
    }

    //set sample rate to 1kHz
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_SMPLRT_DIV, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to set sample rate");
        return ret;
    }

    //Set gyro fsr to 250 dps (max sensitivity)
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to set gyro sensitivity");
        return ret;
    }

    //Set accel fsr to 2g (max sensitivity)
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to set acce sensitivity");
        return ret;
    }

    uint16_t gyro_sensitivity  = 131;
    uint16_t acce_sensitivity = 16384;

    //Configure FIFO to capture acce/gyro data for bias calibration    
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_USER_CTRL, 0x40);
    ret += i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_FIFO_EN, 0x78);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to configure FIFO");
        return ret;
    }

    sleep(1); //accumulate 1000 samples

    //turn off FIFO sensor read
    ret = i2c_bus_write_byte(mpu9250.mpu6500->i2c_dev, MPU6500_FIFO_EN, 0x00);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to turn off FIFO sensor read");
        return ret;
    }

    //read FIFO sample count
    ret = i2c_bus_read_bytes(mpu9250.mpu6500->i2c_dev, MPU6500_FIFO_COUNTH, 2, data);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to read FIFO sample count");
        return ret;
    }

    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    pkt_count = fifo_count/12;
    ESP_LOGI(TAG, "FIFO Count: %d", fifo_count);
    ESP_LOGI(TAG, "Measurement count %d", pkt_count);
    int16_t acce_temp[3] = {0, 0, 0};
    int16_t gyro_temp[3] = {0, 0, 0};

    for (i = 0; i < pkt_count; i++){
        i2c_bus_read_bytes(mpu9250.mpu6500->i2c_dev, MPU6500_FIFO_R_W, 12, data);

        acce_temp[0] = (int16_t)(((int16_t)data[0]<<8)|data[1]);
        acce_temp[1] = (int16_t)(((int16_t)data[2]<<8)|data[3]);
        acce_temp[2] = (int16_t)(((int16_t)data[4]<<8)|data[5]);

        gyro_temp[0] = (int16_t)(((int16_t)data[6]<<8)|data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8]<<8)|data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10]<<8)|data[11]);

        acce_bias[0] += (int32_t)acce_temp[0];
        acce_bias[1] += (int32_t)acce_temp[1];
        acce_bias[2] += (int32_t)acce_temp[2];

        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    acce_bias[0] /= (int32_t) pkt_count;
    acce_bias[1] /= (int32_t) pkt_count;
    acce_bias[2] /= (int32_t) pkt_count;

    gyro_bias[0] /= (int32_t) pkt_count;
    gyro_bias[1] /= (int32_t) pkt_count;
    gyro_bias[2] /= (int32_t) pkt_count;

    //remove gravity from the z-axis accelerometer bias calculation
    if (acce_bias[2] > 0L){
        acce_bias[2] -= (int32_t) acce_sensitivity;
    } else {
        acce_bias[2] += (int32_t) acce_sensitivity;
    }

    //Push gyro biases to the registers which are rest on device startup or reset
    data[0] = (-gyro_bias[0]/4 >> 8) & 0xFF;
    data[1] = (-gyro_bias[0]/4) & 0xFF;
    data[2] = (-gyro_bias[1]/4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4) & 0xFF;
    data[4] = (-gyro_bias[2]/4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4) & 0xFF;

    i2c_bus_write_bytes(mpu9250.mpu6500->i2c_dev, MPU6500_XG_OFFSET_H, 6, data);

    ESP_LOGI(TAG, "acce_x: %d\tacce_y: %d\tacce_z: %d", acce_bias[0], acce_bias[1], acce_bias[2]);
    ESP_LOGI(TAG, "gyro_x: %d\tgyro_y: %d\tgyro_z: %d", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    
    return ESP_OK;
}


//Mag readings from the AK8963
ak8963_handle_t ak8963_create(uint8_t dev_addr)
{
    if (!is_i2c_bus_initialized(SENSOR_BUS)){
        return NULL;
    }

    ak8963_dev_t *sens = (ak8963_dev_t *) calloc(1, sizeof(ak8963_dev_t));
    sens->i2c_dev = i2c_bus_device_create(sensor_bus, dev_addr, i2c_bus_get_current_clk_speed(sensor_bus));
    if (sens->i2c_dev == NULL) {
        ak8963_delete(sens);
        return NULL;
    }

    //Power down magnetometer
    esp_err_t ret = i2c_bus_write_byte(sens->i2c_dev,AK8963_CNTL1, 0x00);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to power down AK8963");
        ak8963_delete(sens);
        return NULL;
    }
    sleep(0.1);

    //Enter Fuse ROM mode
    ret = i2c_bus_write_byte(sens->i2c_dev, AK8963_CNTL1, 0x0F);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to enter fuse ROM mode");
        ak8963_delete(sens);
        return NULL;
    }
    sleep(0.1);

    //Read magnetometer sensitivity adjustment values
    uint8_t asa_val[3];
    ret = i2c_bus_read_bytes(sens->i2c_dev, AK8963_ASA, 3, asa_val);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to read sensitivity adjustment values");
        ak8963_delete(sens);
        return NULL;
    }
    //Save raw magnetometer sensitivity adjustment values
    sens->asa_values.asa_x = (float)asa_val[0];
    sens->asa_values.asa_y = (float)asa_val[1];
    sens->asa_values.asa_z = (float)asa_val[2];

    //Save magnetometer sensitivity adjustment values
    sens->asa_adjust.asa_adjust_x = (((sens->asa_values.asa_x - 128)*0.5)/128.0 + 1.0f);
    sens->asa_adjust.asa_adjust_y = (((sens->asa_values.asa_y - 128)*0.5)/128.0 + 1.0f);
    sens->asa_adjust.asa_adjust_z = (((sens->asa_values.asa_z - 128)*0.5)/128.0 + 1.0f);


    ESP_LOGI(TAG, "Magnetometer Senstivity Adjustment Values\nASA_x: %f\tAsa_y: %f\tASA_z: %f", sens->asa_values.asa_x, sens->asa_values.asa_y, sens->asa_values.asa_z);
    //Power down magnetometer
    ret = i2c_bus_write_byte(sens->i2c_dev,AK8963_CNTL1, 0x00);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to power down AK8963");
        ak8963_delete(sens);
        return NULL;
    }
    sleep(0.1);

    //Enter continuous measurement mode 2 (200 Hz sample rate)
    ret = i2c_bus_write_byte(sens->i2c_dev, AK8963_CNTL1, AK8963_CNT_MEAS1);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to enter continuous measurement mode");
        ak8963_delete(sens);
        return NULL;
    }
    sleep(0.1);
    sens->dev_addr = dev_addr;
    return (ak8963_handle_t) sens;
}

esp_err_t ak8963_delete(ak8963_dev_t *sensor)
{
    if (sensor == NULL) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_bus_device_delete(&sensor->i2c_dev);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to delete ak8963 on sensor bus");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Successfully deleted ak8963 on sensor bus");
    free(sensor);
    return ESP_OK;
}

esp_err_t ak8963_get_deviceid()
{
    if (!is_i2c_bus_initialized(SENSOR_BUS)){
        ESP_LOGE(TAG, "Uninitialized sensor bus");
        return ESP_FAIL;
    }

    if (mpu9250.ak8963 == NULL){
        ESP_LOGE(TAG, "Uninitialized AK8963 device on sensor bus");
        return ESP_FAIL;
    }
    ak8963_dev_t *sens = mpu9250.ak8963;
    uint8_t deviceid;
    esp_err_t ret = i2c_bus_read_byte(sens->i2c_dev, AK8963_WHO_AM_I, &deviceid);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to get device ID from ak8963");
        return ret;
    }
    if (deviceid != 0x48){
        ESP_LOGW(TAG, "Wrong AKxxxx device found on bus. Actual response: 0x%x", deviceid);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Found AK8963 device on bus. Response code: 0x%x", deviceid);
    return ESP_OK;
}

esp_err_t ak8963_get_raw_mag(ak8963_raw_mag_value_t *raw_mag_value)
{
    if (mpu9250.ak8963 == NULL){
        ESP_LOGW(TAG, "Cannot read from uninitialized sensor");
        return ESP_FAIL;
    }
    uint8_t data_rd[7];
    uint8_t drdy;

    esp_err_t ret = i2c_bus_read_byte(mpu9250.ak8963->i2c_dev, AK8963_ST1, &drdy);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to read data ready byte from Magnetometer");
        return ret;
    }

    if (drdy & 0x01){
        ret = i2c_bus_read_bytes(mpu9250.ak8963->i2c_dev, AK8963_HXL, 7, data_rd);
        if (ret == ESP_OK){
            raw_mag_value->raw_mag_x = (int16_t)((data_rd[1] << 8) | (data_rd[0]));
            raw_mag_value->raw_mag_y = (int16_t)((data_rd[3] << 8) | (data_rd[2]));
            raw_mag_value->raw_mag_z = (int16_t)((data_rd[5] << 8) | (data_rd[4]));
        } else {
            return ESP_FAIL;
        }
    }
    return ret;
}

esp_err_t ak8963_get_mag(ak8963_mag_value_t *mag_value)
{
    if (mpu9250.ak8963 == NULL){
        ESP_LOGE(TAG, "Uninitialized MPU6500 device on sensor bus");
        return ESP_FAIL;
    }
    ak8963_raw_mag_value_t raw_mag;
    esp_err_t ret = ak8963_get_raw_mag(&raw_mag);
    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Failed to read raw magnetometer values");
        return ret;
    }

    ak8963_asa_adjust_value_t asa_adjust = mpu9250.ak8963->asa_adjust;
    mag_value->mag_x = raw_mag.raw_mag_x * asa_adjust.asa_adjust_x;
    mag_value->mag_y = raw_mag.raw_mag_y * asa_adjust.asa_adjust_y;
    mag_value->mag_z = raw_mag.raw_mag_z * asa_adjust.asa_adjust_z;
    // ESP_LOGI(TAG, "Mag_x: %f\t\tMag_y: %f\t\tMag_z: %f", mag_value->mag_x, mag_value->mag_y, mag_value->mag_z);
    return ESP_OK;
}

/**
 * Implementation of the IMU HAL Interface
*/
esp_err_t mpu9250_create()
{
    if (!is_i2c_bus_initialized(SENSOR_BUS)){
        ESP_LOGW(TAG, "Failed create MPU9250 device on uninitialized bus");
        return ESP_FAIL;
    }

    mpu9250.mpu6500 = mpu6500_create(MPU6500_I2C_ADDRESS);
    if (mpu9250.mpu6500 ==NULL){
        ESP_LOGW(TAG, "Failed to create device for mpu6500");
        return ESP_FAIL;
    }
    esp_err_t ret = mpu6500_get_acce_sensitivity();
    if (ret != ESP_OK){
        return ret;
    }
    ret = mpu6500_get_gyro_sensitivity();
    if (ret != ESP_OK){
        return ret;
    }
    ESP_LOGI(TAG, "Checking MPU6500 presence on sensor bus");
    mpu6500_get_deviceid();
    mpu9250.ak8963 = ak8963_create(AK8963_I2C_ADDRESS);
    if (mpu9250.ak8963 ==NULL){
        ESP_LOGW(TAG, "Failed to create device for ak8963");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Checking AK8963 presence on sensor bus");
    ak8963_get_deviceid();
    
    mpu9250.counter = 0;
    mpu9250.dt = 0;
    mpu9250.timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    if (mpu9250.timer == NULL){
        ESP_LOGE(TAG, "Failed to dynamically allocate memory to timer struct");
        mpu6500_delete((mpu6500_handle_t) mpu9250.mpu6500);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mpu9250_delete()
{
    if (!is_i2c_bus_initialized(SENSOR_BUS)){
        ESP_LOGW(TAG, "Cannot delete device on an uninitialized bus");
        return ESP_FAIL;
    }
    mpu6500_delete(mpu9250.mpu6500);
    ak8963_delete(mpu9250.ak8963);
    free(mpu9250.timer);
    return ESP_OK;
}

esp_err_t imu_mpu9250_acquire_acce()
{
    esp_err_t ret = mpu6500_get_acce(&acce_values);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    mpu9250.mpu6500->acce_values.acce_x = acce_values.acce_x;
    mpu9250.mpu6500->acce_values.acce_y = acce_values.acce_y;
    mpu9250.mpu6500->acce_values.acce_z = acce_values.acce_z;
    return ESP_OK;
}

esp_err_t imu_mpu9250_acquire_gyro()
{   
    esp_err_t ret = mpu6500_get_gyro(&gyro_values);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    mpu9250.mpu6500->gyro_values.gyro_x = gyro_values.gyro_x;
    mpu9250.mpu6500->gyro_values.gyro_y = gyro_values.gyro_y;
    mpu9250.mpu6500->gyro_values.gyro_z = gyro_values.gyro_z;
    return ESP_OK;
}

esp_err_t imu_mpu9250_acquire_mag()
{
    esp_err_t ret = ak8963_get_mag(&mag_values);
    if (ret != ESP_OK){
        return ESP_FAIL;
    }
    mpu9250.ak8963->mag_values.mag_x = mag_values.mag_x;
    mpu9250.ak8963->mag_values.mag_y = mag_values.mag_y;
    mpu9250.ak8963->mag_values.mag_z = mag_values.mag_z;
    return ESP_OK;
}