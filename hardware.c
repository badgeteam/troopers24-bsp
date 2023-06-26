#include "hardware.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <esp_log.h>

#include "../troopers23-efuse/include/efuse.h"
#include "managed_i2c.h"
#include "pax_gfx.h"
#include "rainbow.h"

static const char* TAG = "hardware";

static ILI9341  dev_ili9341  = {0};
static CC1200   cc1200       = {0};
static Keyboard dev_keyboard = {
    .i2c_bus          = I2C_BUS,
    .intr_pin         = GPIO_INT_KEY,
    .addr_front       = PCA555A_0_ADDR,
    .addr_keyboard1   = PCA555A_1_ADDR,
    .addr_keyboard2   = PCA555A_2_ADDR,
    .pin_sao_presence = IO_SAO_GPIO2,
};
static PCA9555* dev_io_expander = {0};
static Controller dev_controller = {0};
static KTD2052 dev_ktd2052 = {0};

static bool bsp_ready = false;

static xSemaphoreHandle i2c_semaphore = NULL;
static xSemaphoreHandle spi_semaphore = NULL;

static pax_buf_t pax_buffer;

esp_err_t sao_set_leds(uint16_t a, uint16_t b, uint16_t c, uint16_t d) {
    esp_err_t res;

    res = ktd2052_set_color_pax(&dev_ktd2052, 1, rainbow(a, NUM_BADGES));
    if (res != ESP_OK) return res;

    res = ktd2052_set_color_pax(&dev_ktd2052, 2, rainbow(b, NUM_BADGES));
    if (res != ESP_OK) return res;

    res = ktd2052_set_color_pax(&dev_ktd2052, 3, rainbow(c, NUM_BADGES));
    if (res != ESP_OK) return res;

    res = ktd2052_set_color_pax(&dev_ktd2052, 4, rainbow(d, NUM_BADGES));
    if (res != ESP_OK) return res;

    return ESP_OK;
}

static inline void sao_presence_change(bool connected) {
    esp_err_t res;
    if (connected) {
        // Shitty was just connected
        ESP_LOGI(TAG, "SAO: connected");
        int retry = 5;

        // Shot debounce
        while (!ktd2052_connected(&dev_ktd2052) && retry-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        uint16_t id = badge_id();

        res = ktd2052_init(&dev_ktd2052);
        if (res != ESP_OK) goto err;

        res = sao_set_leds(id, id * (NUM_BADGES / 4.), id + 2. * (NUM_BADGES / 4.), id + 3. * (NUM_BADGES / 4.));
        if (res != ESP_OK) goto err;
    } else {
        ESP_LOGI(TAG, "SAO: disconnected");
    }
    return;
err:
    ESP_LOGE(TAG, "Error in communication with LED controller: %d", res);
}

void controller_led_callback() {
    uint8_t buf[8];
    esp_fill_random(buf, 8);
    uint16_t a = (((uint16_t)buf[0] << 8) + buf[1]) % NUM_BADGES;
    uint16_t b = (((uint16_t)buf[2] << 8) + buf[3]) % NUM_BADGES;
    uint16_t c = (((uint16_t)buf[4] << 8) + buf[5]) % NUM_BADGES;
    uint16_t d = (((uint16_t)buf[6] << 8) + buf[7]) % NUM_BADGES;
    sao_set_leds(a, b, c, d);
}

static esp_err_t _bus_init() {
    esp_err_t res;

    // I2C bus
    i2c_config_t i2c_config = {.mode             = I2C_MODE_MASTER,
                               .sda_io_num       = GPIO_I2C_SDA,
                               .scl_io_num       = GPIO_I2C_SCL,
                               .master.clk_speed = I2C_SPEED,
                               .sda_pullup_en    = false,
                               .scl_pullup_en    = false,
                               .clk_flags        = 0};

    res = i2c_param_config(I2C_BUS, &i2c_config);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Configuring I2C bus parameters failed");
        return res;
    }

    res = i2c_set_timeout(I2C_BUS, I2C_TIMEOUT * 80);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Configuring I2C bus timeout failed");
        return res;
    }

    res = i2c_driver_install(I2C_BUS, i2c_config.mode, 0, 0, 0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Initializing system I2C bus failed");
        return res;
    }

    i2c_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_semaphore);

    // SPI bus
    spi_bus_config_t busConfiguration = {0};
    busConfiguration.mosi_io_num      = GPIO_SPI_MOSI;
    busConfiguration.miso_io_num      = GPIO_SPI_MISO;
    busConfiguration.sclk_io_num      = GPIO_SPI_CLK;
    busConfiguration.quadwp_io_num    = -1;
    busConfiguration.quadhd_io_num    = -1;
    busConfiguration.max_transfer_sz  = SPI_MAX_TRANSFER_SIZE;
    res                               = spi_bus_initialize(SPI_BUS, &busConfiguration, SPI_DMA_CHANNEL);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Initializing SPI bus failed");
        return res;
    }

    spi_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(spi_semaphore);

    return ESP_OK;
}

esp_err_t bsp_init() {
    if (bsp_ready) return ESP_OK;

    esp_err_t res;

    // Interrupts
    res = gpio_install_isr_service(0);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Installing ISR service failed");
        return res;
    }

    // Communication busses
    res = _bus_init();
    if (res != ESP_OK) return res;

    // SAO LED controller
    dev_ktd2052.i2c_addr = KTD2052_A_ADDRESS;
    dev_ktd2052.i2c_semaphore = i2c_semaphore;
    res = ktd2052_init(&dev_ktd2052);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Initializing ktd2052 failed");
        return res;
    }

    // Keyboard
    dev_keyboard.i2c_semaphore   = i2c_semaphore;
    dev_keyboard.sao_presence_cb = &sao_presence_change;
    res                          = keyboard_init(&dev_keyboard);
    if (res != ESP_OK) return res;

    // IO expander
    dev_io_expander = dev_keyboard.front;
//    res = pca9555_set_gpio_polarity(dev_io_expander, IO_SAO_GPIO2, PCA_INVERTED);
//    if (!res) {
//        return res;
//    }

    // LCD display
    dev_ili9341.spi_bus               = SPI_BUS;
    dev_ili9341.pin_cs                = GPIO_SPI_CS_LCD;
    dev_ili9341.pin_dcx               = GPIO_SPI_DC_LCD;
    dev_ili9341.pin_reset             = GPIO_LCD_RESET;
    dev_ili9341.rotation              = 1;
    dev_ili9341.color_mode            = true;      // Blue and red channels are swapped
    dev_ili9341.spi_speed             = 20000000;  // 20MHz
    dev_ili9341.spi_max_transfer_size = SPI_MAX_TRANSFER_SIZE;
    dev_ili9341.spi_semaphore = spi_semaphore;
    dev_ili9341.reset_external_pullup = false;

    res = ili9341_init(&dev_ili9341);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Initializing LCD failed");
        return res;
    }

    pax_buf_init(&pax_buffer, NULL, ILI9341_WIDTH, ILI9341_HEIGHT, PAX_BUF_16_565RGB);
    pax_buf_reversed(&pax_buffer, true);

    cc1200.spi_bus   = SPI_BUS;
    cc1200.pin_cs    = GPIO_SPI_CS_RADIO;
    cc1200.pin_intr  = GPIO_INT_RADIO;
    cc1200.spi_speed = 20000000;  // 20MHz
    cc1200.spi_semaphore = spi_semaphore;

    res = cc1200_init(&cc1200);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Initializing CC1200 failed");
        return res;
    }


    dev_controller.i2c_addr = CONTROLLER_ADDRESS;
    dev_controller.i2c_semaphore = i2c_semaphore;
    dev_controller.need_init = false;
    dev_controller.poll_delay = 50;
    dev_controller.queue = dev_keyboard.queue;
    res = controller_init(&dev_controller);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Initializing controller failed");
        return res;
    }

//    uint8_t msg[] = { 0x41, 0x42, 0x43 };
//    while(1) {
//        driver_cc1200_tx_packet(msg, 3);
//        vTaskDelay(2000 / portTICK_PERIOD_MS);
//    }

    bsp_ready = true;
    return ESP_OK;
}

ILI9341* get_ili9341() {
    if (!bsp_ready) return NULL;
    return &dev_ili9341;
}

PCA9555 * get_io_expander() {
    if (!bsp_ready) return NULL;
    return dev_io_expander;
}

Keyboard* get_keyboard() {
    if (!bsp_ready) return NULL;
    return &dev_keyboard;
}

esp_err_t clear_keyboard_queue() {
    xQueueReset(dev_keyboard.queue);
    return ESP_OK;
}

Controller* get_controller() {
    if (!bsp_ready) return NULL;
    return &dev_controller;
}

KTD2052* get_ktd2052() {
    if (!bsp_ready) return NULL;
    return &dev_ktd2052;
}

esp_err_t display_flush() {
    if (!bsp_ready) return ESP_FAIL;
    if (!pax_is_dirty(&pax_buffer)) return ESP_OK;
    // ESP_LOGI(TAG, "Flush %u to %u\n", pax_buffer.dirty_y0, pax_buffer.dirty_y1);
    uint8_t*  buffer = (uint8_t*) (pax_buffer.buf);
    esp_err_t res    = ili9341_write_partial_direct(&dev_ili9341, &buffer[pax_buffer.dirty_y0 * ILI9341_WIDTH * 2], 0, pax_buffer.dirty_y0, ILI9341_WIDTH,
                                                    pax_buffer.dirty_y1 - pax_buffer.dirty_y0 + 1);
    if (res != ESP_OK) return res;
    pax_mark_clean(&pax_buffer);
    return res;
}

pax_buf_t* get_pax_buffer() {
    if (!bsp_ready) return NULL;
    return &pax_buffer;
}
