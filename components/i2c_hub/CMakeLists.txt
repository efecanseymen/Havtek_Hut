idf_component_register(SRCS "i2c_hub.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver esp_driver_i2c esp_driver_gpio)
