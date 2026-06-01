idf_component_register(SRCS "hello_world_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES bt nvs_flash esp_lcd esp_driver_gpio esp_driver_i2c
                       REQUIRES decadriver)
