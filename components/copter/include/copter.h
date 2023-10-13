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
 * @file copter.h
 * @defgroup copter configuration
 * @{
 *
 * ESP-IDF component for copter configuration
 *
 * Copyright (c) 2022 Machar Kook <jkook2012@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __COPTER_H__
#define __COPTER_H__

#include <stdbool.h>
#include <esp_err.h>

#include "esc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Hexacopter {
    motor_config_t M1;
    motor_config_t M2;
    motor_config_t M3;
    motor_config_t M4;
    motor_config_t M5;
    motor_config_t M6;
    bool is_armed;
} hexacopter_config_t;

typedef struct Quadcopter {
    motor_config_t M1;
    motor_config_t M2;
    motor_config_t M3;
    motor_config_t M4;
    bool is_armed;
} quadcopter_config_t;


typedef void *copter_handle_t;

/**
 * @brief Initialize Vehicle configuration and communication bus
 *
 * @return `ESP_OK` on success
 */
esp_err_t init_vehicle(void);

/**
 * @brief Deinitialize Vehicle configuration and communication bus
 *
 * @return `ESP_OK` on success
 */
void deinit_vehicle(void);

/**
 * @brief Log copter information to serial console
*/
void copter_info(void);

/**
 * @brief Arm all Motors/ESCs
 * @return `ESP_OK` on success
 */
esp_err_t arm_motors(void);

/**
 * @brief Disarm all Motors/ESCs
 * @return `ESP_OK` on success
 */
esp_err_t disarm_motors(void);

/**
 * @brief Calibrate multiple ESCs
 * @return `ESP_OK` on success
 */
esp_err_t calibrate_escs(void);

/**
 * @brief Set single motor throttle for a copter configuration
 * @param throttle_value Array of throttle value as a percentage (each)
 * @return `ESP_OK` on success
 */
esp_err_t set_throttle_copter(float throttle_value);

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __COPTER_H__ */