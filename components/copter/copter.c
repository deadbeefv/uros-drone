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
 * @file hexacopter.c
 * @defgroup hexacopter configuration
 * @{
 *
 * ESP-IDF component for hexacopter configuration
 *
 * Copyright (c) 2022 Machar Kook <jkook2012@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_err.h>

#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "copter.h"

#define OE_PIN GPIO_NUM_18
#define LOW 0
#define HIGH 1

#ifdef CONFIG_HEXACOPTER
    static const char *TAG = "Hexacopter";
    const uint16_t arm_values[] = {arm_value, arm_value, arm_value, arm_value, arm_value, arm_value};
    const uint16_t disarm_values[] = {0, 0, 0, 0, 0, 0};
    uint16_t throttle[] = {0, 0, 0, 0, 0, 0};

    #define MOTOR_COUNT 6

    hexacopter_config_t hexacopter = {
                    {CONFIG_M1, false}, 
                    {CONFIG_M2, false},
                    {CONFIG_M3, false}, 
                    {CONFIG_M4, false},
                    {CONFIG_M5, false}, 
                    {CONFIG_M6, false}, 
                    false};
    copter_handle_t copter;

    esp_err_t init_vehicle()
    {
        esp_err_t ret = init_esc();
        if (ret != ESP_OK){
            return ret;
        }
        ret = gpio_set_direction(OE_PIN, GPIO_MODE_OUTPUT);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Direction on Output Enable Pin %d", OE_PIN);
            return ret;
        }

        copter = (void *) &hexacopter;
        ret = disarm_motors();
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Failed to Disarm Motors");
            return ret;
        }
        ESP_LOGI(TAG, "Hexacopter initialized successfully");
        return ESP_OK;
    }

    void deinit_vehicle(void)
    {
        esp_err_t ret = disarm_motors();
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Failed to disarm motors");
        }
        copter = NULL;
    }

    void copter_info(){
        if (copter == NULL){
            ESP_LOGW(TAG, "Uninitialized Motors");
        } else {
            hexacopter_config_t * hexacopter = (hexacopter_config_t *) copter;
            ESP_LOGI(TAG, "M1 connected to channel %d. Arm status: %d", hexacopter->M1.channel, hexacopter->M1.is_armed);
            ESP_LOGI(TAG, "M2 connected to channel %d. Arm status: %d", hexacopter->M2.channel, hexacopter->M2.is_armed);
            ESP_LOGI(TAG, "M3 connected to channel %d. Arm status: %d", hexacopter->M3.channel, hexacopter->M3.is_armed);
            ESP_LOGI(TAG, "M4 connected to channel %d. Arm status: %d", hexacopter->M4.channel, hexacopter->M4.is_armed);
            ESP_LOGI(TAG, "M5 connected to channel %d. Arm status: %d", hexacopter->M5.channel, hexacopter->M5.is_armed);
            ESP_LOGI(TAG, "M6 connected to channel %d. Arm status: %d", hexacopter->M6.channel, hexacopter->M6.is_armed);

            ESP_LOGI(TAG, "Hexacopter Arm status: %d", hexacopter->is_armed);
        }
    }

    esp_err_t arm_motors(void)
    {
        if (copter == NULL){
            ESP_LOGW(TAG, "Cannot Arm Uninitialized Motors");
            return ESP_FAIL;
        }

        hexacopter_config_t * hexacopter = (hexacopter_config_t *) copter;

        //if hexacopter is already armed, return ESP_OK
        if (hexacopter->is_armed){
            ESP_LOGI(TAG, "Hexacopter is already armed!");
            return ESP_OK;
        }
        
        esp_err_t ret = gpio_set_level(OE_PIN, LOW);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level on Output Enable Pin %d", OE_PIN);
            return ret;
        }

        motor_config_t motor = hexacopter->M1;
        ret = pca9685_set_pwm_values(motor.channel, MOTOR_COUNT, arm_values);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "Could not Arm Motors");
            //TODO: SET PCA9685 OE pin High
            return ret;
        }
        //set motor arm flag as true
        hexacopter->M1.is_armed = true;
        hexacopter->M2.is_armed = true;
        hexacopter->M3.is_armed = true;
        hexacopter->M4.is_armed = true;
        hexacopter->M5.is_armed = true;
        hexacopter->M6.is_armed = true;
        //Wait for the motors to arm
        sleep(ESC_CALIBRATE_DELAY_HIGH/1000);
        // vTaskDelay(ESC_CALIBRATE_DELAY_HIGH/portTICK_PERIOD_MS);
        hexacopter->is_armed = true;
        ESP_LOGI(TAG, "Arm successful");
        return ESP_OK;
    }

    esp_err_t disarm_motors(void)
    {
        if (copter == NULL){
            ESP_LOGE(TAG, "Cannot Disarm Uninitialized Motors");
            return ESP_FAIL;
        }

        hexacopter_config_t * hexacopter = (hexacopter_config_t *) copter;
        
        if (!hexacopter->is_armed){
            ESP_LOGI(TAG, "Cannot disarm disarmed motors");
            return ESP_OK;
        }

        esp_err_t ret = set_throttle_copter(0);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could not lower throttle to zero");
            return ret;
        }
        
        ret = gpio_set_level(OE_PIN, HIGH);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level HIGH on Output Enable Pin %d", OE_PIN);
            return ret;
        }  

        ret = pca9685_set_pwm_values(hexacopter->M1.channel, MOTOR_COUNT, disarm_values);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "Could not Arm Motors");
            //TODO: SET PCA9685 OE pin High
            return ret;
        }
        //unset motor arm flag as true
        hexacopter->M1.is_armed = false;
        hexacopter->M2.is_armed = false;
        hexacopter->M3.is_armed = false;
        hexacopter->M4.is_armed = false;
        hexacopter->M5.is_armed = false;
        hexacopter->M6.is_armed = false;
        hexacopter->is_armed = false;
        return ESP_OK;
    }

    esp_err_t calibrate_escs(void)
    {
        ESP_LOGI(TAG, "Calibrating Motors");
        if (copter == NULL){
            ESP_LOGW(TAG, "Cannot Calibrate Uninitialized Motors");
            return ESP_FAIL;
        } 
        
        esp_err_t ret = gpio_set_level(OE_PIN, LOW);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level LoW on Output Enable Pin %d", OE_PIN);
            return ret;
        }    
        
        ret = set_throttle_copter(CONFIG_THROTTLE_MAX_VALUE);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "ESC Calibration Failed");
            return ESP_FAIL;
        }
        sleep(ESC_CALIBRATE_DELAY_LOW/1000);
        // vTaskDelay(ESC_CALIBRATE_DELAY_LOW/portTICK_PERIOD_MS);

        ret = set_throttle_copter(CONFIG_THROTTLE_MIN_VALUE);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "ESC Calibration Failed");
            return ESP_FAIL;
        }
        sleep(ESC_CALIBRATE_DELAY_HIGH/1000);
        // vTaskDelay(ESC_CALIBRATE_DELAY_HIGH/portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ESC calibration successful");

        ret = gpio_set_level(OE_PIN, HIGH);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level HIGH on Output Enable Pin %d", OE_PIN);
            return ret;
        }  
        return ESP_OK;
    }

    esp_err_t set_throttle_copter(float throttle_value)
    {
        if (copter == NULL){
            ESP_LOGW(TAG, "Cannot Throttle Uninitialized Motors");
            return ESP_FAIL;
        }

        // if (throttle_value < CONFIG_THROTTLE_MIN_VALUE || throttle_value > CONFIG_THROTTLE_MAX_VALUE){
        //     ESP_LOGW(TAG, "Throttle value of %02f above %d or below %d", throttle_value, CONFIG_THROTTLE_MAX_VALUE, CONFIG_THROTTLE_MIN_VALUE);
        //     return ESP_FAIL;
        // }

        hexacopter_config_t *hexacopter = (hexacopter_config_t *) copter;
        if (!hexacopter->is_armed){
            ESP_LOGW(TAG, "Cannot Throttle Unarmed Motors");
            return ESP_FAIL;
        }
        
        motor_config_t M1 = hexacopter->M1;
        uint16_t mapped_throttle = map(throttle_value, CONFIG_THROTTLE_MIN_VALUE, CONFIG_THROTTLE_MAX_VALUE, CONFIG_ESC_MIN_VALUE, CONFIG_ESC_MAX_VALUE);

        for (int i = 0; i < MOTOR_COUNT; i++){
            throttle[i] = mapped_throttle;
            // ESP_LOGI(TAG, "M%d Throttle Mapped to %d", i+1, throttle[i]);
        }

        esp_err_t ret = pca9685_set_pwm_values(M1.channel, MOTOR_COUNT, throttle);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "Could not set throttle");
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    
#endif
#ifdef CONFIG_QUADCOPTER
    static const char *TAG = "Quadcopter";
    const uint16_t arm_values[] = {arm_value, arm_value, arm_value, arm_value};
    const uint16_t disarm_values[] = {0, 0, 0, 0};
    uint16_t throttle[] = {0, 0, 0, 0};

    #define MOTOR_COUNT 4

    quadcopter_config_t quadcopter = {
                    {CONFIG_M1, false}, 
                    {CONFIG_M2, false},
                    {CONFIG_M3, false}, 
                    {CONFIG_M4, false},
                    false};
    copter_handle_t copter;

    esp_err_t init_vehicle()
    {
        esp_err_t ret = init_esc();
        if (ret != ESP_OK){
            return ret;
        }
        ret = gpio_set_direction(OE_PIN, GPIO_MODE_OUTPUT);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Direction on Output Enable Pin %d", OE_PIN);
            return ret;
        }

        copter = (void *) &quadcopter;
        ret = disarm_motors();
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Failed to Disarm Motors");
            return ret;
        }
        ESP_LOGI(TAG, "Quadcopter initialized successfully");
        return ESP_OK;
    }

    void deinit_vehicle(void)
    {
        esp_err_t ret = disarm_motors();
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Failed to disarm motors");
        }
        copter = NULL;
    }

    void copter_info(){
        if (copter == NULL){
            ESP_LOGW(TAG, "Uninitialized Motors");
        } else {
            quadcopter_config_t * quadcopter = (quadcopter_config_t *) copter;
            ESP_LOGI(TAG, "M1 connected to channel %d. Arm status: %d", quadcopter->M1.channel, quadcopter->M1.is_armed);
            ESP_LOGI(TAG, "M2 connected to channel %d. Arm status: %d", quadcopter->M2.channel, quadcopter->M2.is_armed);
            ESP_LOGI(TAG, "M3 connected to channel %d. Arm status: %d", quadcopter->M3.channel, quadcopter->M3.is_armed);
            ESP_LOGI(TAG, "M4 connected to channel %d. Arm status: %d", quadcopter->M4.channel, quadcopter->M4.is_armed);

            ESP_LOGI(TAG, "Quadcopter Arm status: %d", quadcopter->is_armed);
        }
    }

    esp_err_t arm_motors(void)
    {
        if (copter == NULL){
            ESP_LOGW(TAG, "Cannot Arm Uninitialized Motors");
            return ESP_FAIL;
        }

        quadcopter_config_t * quadcopter = (quadcopter_config_t *) copter;

        //if quadcopter is already armed, return ESP_OK
        if (quadcopter->is_armed){
            ESP_LOGI(TAG, "Quadcopter is already armed!");
            return ESP_OK;
        }
        copter_info();
        
        esp_err_t ret = gpio_set_level(OE_PIN, LOW);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level on Output Enable Pin %d", OE_PIN);
            return ret;
        }

        motor_config_t motor = quadcopter->M1;
        ret = pca9685_set_pwm_values(motor.channel, MOTOR_COUNT, arm_values);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "Could not Arm Motors");
            //TODO: SET PCA9685 OE pin High
            return ret;
        }
        //set motor arm flag as true
        quadcopter->M1.is_armed = true;
        quadcopter->M2.is_armed = true;
        quadcopter->M3.is_armed = true;
        quadcopter->M4.is_armed = true;
        //Wait for the motors to arm
        sleep(ESC_CALIBRATE_DELAY_HIGH/1000);
        // vTaskDelay(ESC_CALIBRATE_DELAY_HIGH/portTICK_PERIOD_MS);
        quadcopter->is_armed = true;
        ESP_LOGI(TAG, "Arm successful");
        copter_info();
        return ESP_OK;
    }

    esp_err_t disarm_motors(void)
    {
        if (copter == NULL){
            ESP_LOGE(TAG, "Cannot Disarm Uninitialized Motors");
            return ESP_FAIL;
        }

        quadcopter_config_t * quadcopter = (quadcopter_config_t *) copter;
        
        if (!quadcopter->is_armed){
            ESP_LOGI(TAG, "Cannot disarm disarmed motors");
            return ESP_OK;
        }

        esp_err_t ret = set_throttle_copter(0);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could not lower throttle to zero");
            return ret;
        }

        ret = gpio_set_level(OE_PIN, HIGH);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level HIGH on Output Enable Pin %d", OE_PIN);
            return ret;
        }  


        ret = pca9685_set_pwm_values(quadcopter->M1.channel, MOTOR_COUNT, disarm_values);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "Could not Arm Motors");
            //TODO: SET PCA9685 OE pin High
            return ret;
        }
        //unset motor arm flag as true
        quadcopter->M1.is_armed = false;
        quadcopter->M2.is_armed = false;
        quadcopter->M3.is_armed = false;
        quadcopter->M4.is_armed = false;
        quadcopter->is_armed = false;
        ESP_LOGI(TAG, "Disarm Successful");
        return ESP_OK;
    }

    esp_err_t calibrate_escs(void)
    {
        ESP_LOGI(TAG, "Calibrating Motors");
        if (copter == NULL){
            ESP_LOGW(TAG, "Cannot Calibrate Uninitialized Motors");
            return ESP_FAIL;
        } 
        
        esp_err_t ret = gpio_set_level(OE_PIN, LOW);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level LOW on Output Enable Pin %d", OE_PIN);
            return ret;
        }    
        
        ret = set_throttle_copter(CONFIG_THROTTLE_MAX_VALUE);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "ESC Calibration Failed");
            return ESP_FAIL;
        }
        sleep(ESC_CALIBRATE_DELAY_LOW/1000);
        // vTaskDelay(ESC_CALIBRATE_DELAY_LOW/portTICK_PERIOD_MS);

        ret = set_throttle_copter(CONFIG_THROTTLE_MIN_VALUE);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "ESC Calibration Failed");
            return ESP_FAIL;
        }
        sleep(ESC_CALIBRATE_DELAY_HIGH/1000);
        // vTaskDelay(ESC_CALIBRATE_DELAY_HIGH/portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "ESC calibration successful");

        ret = gpio_set_level(OE_PIN, HIGH);
        if (ret != ESP_OK){
            ESP_LOGE(TAG, "Could Not Set Level HIGH on Output Enable Pin %d", OE_PIN);
            return ret;
        }  
        return ESP_OK;
    }

    esp_err_t set_throttle_copter(float throttle_value)
    {
        if (copter == NULL){
            ESP_LOGW(TAG, "Cannot Throttle Uninitialized Motors");
            return ESP_FAIL;
        }

        if (throttle_value < CONFIG_THROTTLE_MIN_VALUE || throttle_value > CONFIG_THROTTLE_MAX_VALUE){
            ESP_LOGW(TAG, "Throttle values above %d or below %d", CONFIG_THROTTLE_MAX_VALUE, CONFIG_THROTTLE_MIN_VALUE);
            return ESP_FAIL;
        }

        quadcopter_config_t *quadcopter = (quadcopter_config_t *) copter;
        if (!quadcopter->is_armed){
            ESP_LOGW(TAG, "Cannot Throttle Unarmed Motors");
            return ESP_FAIL;
        }
        motor_config_t M1 = quadcopter->M1;;

        uint16_t mapped_throttle = map(throttle_value, CONFIG_THROTTLE_MIN_VALUE, CONFIG_THROTTLE_MAX_VALUE, CONFIG_ESC_MIN_VALUE, CONFIG_ESC_MAX_VALUE);

        for (int i = 0; i < MOTOR_COUNT; i++){
            throttle[i] = mapped_throttle;
        }
        esp_err_t ret = pca9685_set_pwm_values(M1.channel, MOTOR_COUNT, throttle);
        if (ret != ESP_OK){
            ESP_LOGW(TAG, "Could not set throttle");
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    
#endif
