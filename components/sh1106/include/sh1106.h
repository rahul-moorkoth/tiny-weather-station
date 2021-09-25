/* Copyright 2021 Winfried Klum
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Based on work by
 * https://github.com/yanbe/
 */
#ifndef MAIN_SH1106_H_
#define MAIN_SH1106_H_

#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"


#define OLED_I2C_ADDRESS   0x3C

#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

#define OLED_CMD_NOP                    0xE3

#define OLED_CMD_SET_CONTRAST           0x81 

#define OLED_CMD_ENTIRE_DISPLAY_ON      0xA5
#define OLED_CMD_ENTIRE_DISPLAY_OFF     0xA4

#define OLED_CMD_DISPLAY_INVERT_NORMAL   0xA6
#define OLED_CMD_DISPLAY_INVERT_INVERTED 0xA7

#define OLED_CMD_DISPLAY_OFF             0xAE
#define OLED_CMD_DISPLAY_ON              0xAF

#define OLED_CMD_SET_DISPLAY_START_LINE 0x40

#define OLED_CMD_SET_SEGMENT_REMAP_INVERSE 0xA1
#define OLED_CMD_SET_SEGMENT_REMAP_NORMAL 0xA0

#define OLED_CMD_SET_COM_SCAN_MODE_REVERSE 0xC8    
#define OLED_CMD_SET_COM_SCAN_MODE_NORMAL  0xC0
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3

#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5 
#define OLED_CMD_SET_PRECHARGE          0xD9 
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB 

#define OLED_CMD_SET_CHARGE_PUMP_CTRL   0xAD
#define OLED_CMD_SET_CHARGE_PUMP_ON     0x0B
#define OLED_CMD_SET_CHARGE_PUMP_OFF    0x0A

#define tag "SH1106"

void sh1106_init();

void sh1106_display_clear(void *ignore) ;

void sh1106_contrast(uint8_t contrast);

void sh1106_display_text(const void *arg_text);

#endif /* MAIN_SH1106_H_ */
