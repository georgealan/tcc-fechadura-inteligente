idf_component_register(SRCS "wifi_manager.c" "hud_tft.c" "fechadura_fsm.c" "audio_i2s.c" "porta.c" "camera_uvc.c" "olho_magico.c" "display_tft.c" "sensor_mpu.c" "sensor_tof.c" "i2c_bus.c" "teclado.c" "main.c"
                       PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_i2c esp_driver_spi esp_timer esp_driver_i2s esp_wifi esp_event esp_netif nvs_flash
                       INCLUDE_DIRS "")
