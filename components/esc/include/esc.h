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
 * @file esc.h
 * @defgroup brushless motor esc
 * @{
 *
 * ESP-IDF driver for a brushless motor esc
 *
 * Copyright (c) 2022 Machar Kook <jkook2012@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef _ESC_H__
#define __ESC_H__

#include <stdbool.h>
#include <esp_err.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "pca9685.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEFAULT_PRESCALER 23
#define DEFAULT_PWM_FREQUENCY 255

#define ESC_CALIBRATE_DELAY_LOW 1000
#define ESC_CALIBRATE_DELAY_HIGH 2000

typedef  enum  {
    min_value = CONFIG_ESC_MIN_VALUE,
    max_value = CONFIG_ESC_MAX_VALUE,
    arm_value = CONFIG_ESC_ARM_VALUE,
} esc_config_t;

typedef struct  ESC {
    pca9685_channel_t channel;
    bool is_armed;
} motor_config_t;

/**
* @brief Map throttle values from percentage to ESC PWM max_values
* @param x Throttle input 
* @param i_start Input interval min value 
* @param i_end Input interrval max value
* @param o_start Output interval min value 
* @param o_end Output interrval max value 
*
* @return uint16_t throttle value on success
*/
uint16_t map(float x, float i_start, float i_end, uint16_t o_start, uint16_t o_end);

/**
 * @brief Initialize PCA9865 device on I2C bus
 *
 * @return `ESP_OK` on success
 */
esp_err_t init_esc();

/**
 * @brief Arm single Motor/ESC
 * @param motor_config_t Pointer to an ESC connected to a particular channel
 * @return `ESP_OK` on success
 */
esp_err_t arm_motor(motor_config_t *motor);

/**
 * @brief Calibrate single ESC
 * @param motor_config_t Pointer to an ESC connected to a particular channel
 * @return `ESP_OK` on success
 */
esp_err_t calibrate_esc(motor_config_t *motor);

/**
 * @brief Set single motor throttle by adjusting the pwm value
 * @param motor Pointer to an ESC channel to adjust throttle value
 * @param throttle Throttle value as a percentage
 * @return `ESP_OK` on success
 */
esp_err_t set_throttle(motor_config_t *motor, float throttle);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __ESC_H__ */