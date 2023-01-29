#ifndef __BUSCFG_H_
#define __BUSCFG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

typedef enum {
    SENSOR_BUS = 0,
    ACTUATOR_BUS = 1,
} i2c_bus_cfg_t;

/**
 * @brief Check if i2c bus has been initialized
 * 
 * @param bus_cfg i2c_bus config object
 * 
 * @return 
 *      true if i2c bus has been initialized, otherwise false
*/
bool is_i2c_bus_initialized(i2c_bus_cfg_t bus_cfg);

/**
 * @brief Initialize I2C bus
 * 
 * @param bus_cfg i2c_bus config object
 * 
 * @return
 *      ESP_OK if successful, otherwise ESP_FAIL
*/
esp_err_t i2c_bus_init(i2c_bus_cfg_t bus_cfg);

/**
 * @brief Introspect i2c devices connected to bus
 * 
 * @param bus_cfg i2c bus to scan for devices
 * 
 * @return 
 *      ESP_OK if bus scan successful
 *      ESP_FAIL if bus scan failed
*/
esp_err_t scan_bus(i2c_bus_cfg_t bus_cfg);
#ifdef __cplusplus
}
#endif

#endif
