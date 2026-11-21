idf_component_register(SRCS "porta.c" "camera_uvc.c" "olho_magico.c" "display_tft.c" "sensor_mpu.c" "sensor_tof.c" "i2c_bus.c" "teclado.c" "main.c"
                       PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c esp_driver_spi esp_timer
                       INCLUDE_DIRS "")
