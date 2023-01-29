#include <stdbool.h>
#include <stdlib.h>

#include "driver/i2c.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "bus_config.h"

#define MAX_DEVICES 15
static const char *TAG = "bus_config";

i2c_bus_handle_t sensor_bus;
i2c_bus_handle_t actuator_bus;

bool is_i2c_bus_initialized(i2c_bus_cfg_t bus_cfg)
{
    switch (bus_cfg)
    {
    case SENSOR_BUS:
        if (sensor_bus == NULL){
            ESP_LOGW(TAG, "Cannot scan uninitialized sensor bus");
            return false;
        }
        return true;
    case ACTUATOR_BUS:
        if (actuator_bus == NULL){
            ESP_LOGW(TAG, "Cannot scan uninitialized actuator bus");
            return false;
        }
        return true;
    }
    return false;
}

esp_err_t i2c_bus_init(i2c_bus_cfg_t bus_cfg)
{
    switch (bus_cfg)
    {
        case SENSOR_BUS:
            if (sensor_bus != NULL){
                ESP_LOGI(TAG, "Sensor bus already initiated");
                return ESP_OK;
            }
            ESP_LOGI(TAG, "Initializing Sensor Bus for Communication");
            const int i2c_sensor_master_port = I2C_NUM_0;
            const i2c_config_t sensor_conf = {
                .mode = I2C_MODE_MASTER,	
                .sda_io_num = CONFIG_BUS0_I2C_MASTER_SDA_IO,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = CONFIG_BUS0_I2C_MASTER_SCL_IO,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = CONFIG_BUS0_I2C_MASTER_FREQ_HZ,
                .clk_flags = 0,
            };

            sensor_bus = i2c_bus_create(i2c_sensor_master_port, &sensor_conf);
            if (sensor_bus == NULL){
                ESP_LOGE(TAG, "Could not create I2C Sensor Bus");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Sensor Bus Initialized Successfully");
            break;
        
        case ACTUATOR_BUS:
            if (actuator_bus != NULL){
                ESP_LOGI(TAG, "Actuator bus already initiated");
                return ESP_OK;
            }
            ESP_LOGI(TAG, "Initializing Actuator Bus for Communication");
            const int i2c_actuator_master_port = I2C_NUM_1;
            const i2c_config_t actuator_conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = CONFIG_BUS1_I2C_MASTER_SDA_IO,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = CONFIG_BUS1_I2C_MASTER_SCL_IO,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = CONFIG_BUS1_I2C_MASTER_FREQ_HZ,
                .clk_flags = 0,
            };
            actuator_bus = i2c_bus_create(i2c_actuator_master_port, &actuator_conf);
            if (actuator_bus == NULL){
                ESP_LOGE(TAG, "Could not create I2C Actuator Bus");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "Actuator Bus Initialized Successfully");
            break;
        }
    return ESP_OK;
}

esp_err_t scan_bus(i2c_bus_cfg_t bus_cfg)
{
    if (!is_i2c_bus_initialized(bus_cfg)){
        return ESP_FAIL;
    }

    uint8_t *devices = (uint8_t *)calloc(MAX_DEVICES, sizeof(uint8_t));
    if (devices == NULL){
        ESP_LOGW(TAG, "Failed to locate memory for bus scan");
        return ESP_ERR_NO_MEM;
    }
    uint8_t num = 0;
    switch (bus_cfg){
        case SENSOR_BUS:
            num = i2c_bus_scan(sensor_bus, devices, MAX_DEVICES);
            ESP_LOGI(TAG, "Sensor bus scan results:");
            break;
        
        case ACTUATOR_BUS:
            num = i2c_bus_scan(actuator_bus, devices, MAX_DEVICES);
            ESP_LOGI(TAG, "Actuator bus scan results:");
            break;
    }
    for (int i = 0; i < num; i++){
        ESP_LOGI(TAG, "Device %d with address 0x%x found on bus", i+1, devices[i]);
    }
    free(devices);
    return ESP_OK;
}