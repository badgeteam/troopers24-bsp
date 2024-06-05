/*
 * MCH2022 badge
 *
 * This file describes the connections from the ESP32 to the other
 * components on the MCH2022 badge.
 *
 * More information can be found in the documentation at
 * https://www.badge.team/docs/badges/mch2022/pinout
 *
 */

#pragma once

#define REV 1

#define NUM_LEDS 15
#define NUM_BADGES 350

#define GPIO_I2S_BCLK     26
#define GPIO_I2S_WS       12
#define GPIO_I2S_DATA     25

#define GPIO_SD_D0        2
#define GPIO_SD_CLK       14
#define GPIO_SD_CMD       15

#define GPIO_SPI_CLK      18
#define GPIO_SPI_MOSI     23
#define GPIO_SPI_MISO     35
#define GPIO_SPI_CS_LCD   19
#define GPIO_SPI_DC_LCD   13

#define GPIO_I2C_SDA      4
#define GPIO_I2C_SCL      5

#define GPIO_LCD_RESET    21
#ifdef TR23
#define GPIO_LCD_BL       13
#endif

#define GPIO_LED_DATA     22

#define GPIO_INT_KEY      39
#define GPIO_INT_RADIO    34

// I2C bus
#define I2C_BUS        0
#define I2C_SPEED      400000  // 400 kHz
#define I2C_TIMEOUT    250 // us

#define PCA555A_0_ADDR 0x20  // Front-panel
#define PCA555A_1_ADDR 0x21  // Keyboard 1
#define PCA555A_2_ADDR 0x22  // Keyboard 2

// SPI bus
#define SPI_BUS               VSPI_HOST
#define SPI_MAX_TRANSFER_SIZE 4094
#define SPI_DMA_CHANNEL       2
#define SPI_SPEED             2000000  // 2MHz

// IO expander
#define IO_CC_RESET     0x08
#define IO_SD_DETECT    0x0A
#define IO_SAO_GPIO2    0x0B

#define IO_BACKLIGHT  9
#define IO_SAO_DETECT 10
#define IO_DEBUG_LED  11
#define IO_AMP_ENABLE 12
#define IO_AMP_GAIN0  13
#define IO_AMP_GAIN1  14
#define IO_HP_SENSE   15