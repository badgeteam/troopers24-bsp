idf_component_register(
    SRCS "hardware.c"
         "wifi_connection.c"
         "wifi_connect.c"
        "rainbow.c"
    INCLUDE_DIRS "." "include"
    REQUIRES
        "bus-i2c"
        "spi-ili9341"
        "spi-st77xx"
        "keyboard"
        "wpa_supplicant"
        "nvs_flash"
        "pax-graphics"
        "i2c-controller"
        "i2c-ktd2052"
        "spi-st25r3911b"
)
