/*
 * Copyright (c) 2022 Machar Kook <jkook2012@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file esc.c
 * @defgroup brushless motor esc
 * @{
 *
 * ESP-IDF driver for a brushless motor esc
 *
 * Copyright (c) 2022 Machar Kook <jkook2012@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdbool.h>
#include <unistd.h>
#include <esp_err.h>

#include "esp_log.h"
#include "pca9685.h"
#include "esc.h"
#include "bus_config.h"

static const char *TAG = "esc";
extern i2c_bus_handle_t actuator_bus;
uint16_t map(float x, float i_start, float i_end, uint16_t o_start, uint16_t o_end)
{
    return (uint16_t)(x - i_start)/(i_end - i_start)*(o_end - o_start) + o_start;
}

esp_err_t init_esc()
{
    if (actuator_bus == NULL){
        ESP_LOGE(TAG, "Failed to Initialize device on Actuator Bus");
        return ESP_FAIL;
    }

    esp_err_t ret = pca9685_init();
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Failed to Initialize PCA9685");
        return ret;
    }
    ESP_LOGI(TAG, "Initiated PCA9685 successfully");

    ret = pca9685_set_prescaler(DEFAULT_PRESCALER);
        
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Could not get Prescaler Value from PCA9685");
        return ret;
    }
        
    ESP_LOGI(TAG, "Device prescaler set to %d", DEFAULT_PRESCALER);

    ret = pca9685_set_pwm_frequency(DEFAULT_PWM_FREQUENCY);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Could not get PWM Frequency Value from PCA9685");
        return ret;
    }

    ESP_LOGI(TAG, "Device frequency set to %d", DEFAULT_PWM_FREQUENCY);
    return ESP_OK;
}

esp_err_t arm_motor(motor_config_t *motor)
{
    if (motor == NULL){
    ESP_LOGW(TAG, "Cannot arm uninitialized esc on channel %d", motor->channel);
    return ESP_FAIL;
    }
    esp_err_t ret = pca9685_set_pwm_value(motor->channel, arm_value);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Cannot Arm ESC/Motor on Channel %d", motor->channel);
        return ret;
    }
    sleep(ESC_CALIBRATE_DELAY_HIGH/1000);
    // vTaskDelay(ESC_CALIBRATE_DELAY_HIGH/portTICK_PERIOD_MS);
    motor->is_armed = true;
    return ESP_OK;
}

esp_err_t calibrate_esc(motor_config_t *motor)
{
    if (motor == NULL){
        ESP_LOGW(TAG, "Cannot calibrate esc on channel %d", motor->channel);
        return ESP_FAIL;
    }

    esp_err_t ret = set_throttle(motor, CONFIG_THROTTLE_MAX_VALUE);
    if (ret != ESP_OK){
        return ret;
    }
    sleep(ESC_CALIBRATE_DELAY_LOW/1000);
    // vTaskDelay(ESC_CALIBRATE_DELAY_LOW/portTICK_PERIOD_MS);

    ret = set_throttle(motor, CONFIG_THROTTLE_MIN_VALUE);
    if (ret != ESP_OK){
        return ret;
    }
    sleep(ESC_CALIBRATE_DELAY_HIGH/1000);
    // vTaskDelay(ESC_CALIBRATE_DELAY_HIGH/portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t set_throttle(motor_config_t *motor, float throttle)
{
    if (motor == NULL){
        ESP_LOGW(TAG, "Unconfigured ESC on channel %d", motor->channel);
        return ESP_FAIL;
    }

    if (throttle < CONFIG_THROTTLE_MIN_VALUE || throttle > CONFIG_THROTTLE_MAX_VALUE){
        ESP_LOGW(TAG, "Throttle values above %d or below %d", CONFIG_THROTTLE_MAX_VALUE, CONFIG_THROTTLE_MIN_VALUE);
        return ESP_FAIL;
    }

    uint16_t output = map(throttle, CONFIG_THROTTLE_MIN_VALUE, CONFIG_THROTTLE_MAX_VALUE, CONFIG_ESC_MIN_VALUE, CONFIG_ESC_MAX_VALUE);
    esp_err_t ret = pca9685_set_pwm_value(motor->channel, output);
    if (ret != ESP_OK){
        ESP_LOGW(TAG, "Could not set throttle value");
        return ret;
    }
    return ESP_OK;
}