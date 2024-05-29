#pragma once

#include <driver/spi_master.h>
#include <esp_err.h>
#include <sdkconfig.h>

#include "controller.h"
#include "ili9341.h"
#include "keyboard.h"
#include "ktd2052.h"
#include "pax_gfx.h"
#include "troopers23_badge.h"

/** \brief Initialize basic board support
 *
 * \details This function installs the GPIO ISR (interrupt service routine) service
 *          which allows for per-pin GPIO interrupt handlers. Then it initializes the
 *          I2C and SPI communication busses and the LCD display driver. Returns ESP_OK
 *          on success and a
 *
 * \retval ESP_OK   The function successfully executed
 * \retval ESP_FAIL The function failed, possibly indicating hardware failure
 *
 * Check the esp_err header file from the ESP-IDF for a complete list of error codes
 * returned by SDK functions.
 */

void controller_led_callback();

esp_err_t bsp_init();

ILI9341* get_ili9341();

PCA9555 * get_io_expander();

Keyboard* get_keyboard();

Controller* get_controller();

KTD2052* get_ktd2052();

esp_err_t clear_keyboard_queue();

esp_err_t display_flush();
pax_buf_t* get_pax_buffer();