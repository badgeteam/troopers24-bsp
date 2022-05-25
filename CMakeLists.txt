idf_component_register(
    SRCS "hardware.c"
    INCLUDE_DIRS "." "include"
    REQUIRES
        "i2c-bno055"
        "i2c-bme680"
        "spi-ice40"
        "spi-ili9341"
        "mch2022-rp2040"
)
