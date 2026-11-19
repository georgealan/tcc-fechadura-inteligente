idf_component_register(SRCS "sensor_mpu.c" "sensor_tof.c" "i2c_bus.c" "teclado.c" "main.c"
                       PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c
                       INCLUDE_DIRS "")
