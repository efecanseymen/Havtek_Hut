idf_component_register(SRCS "imu.c"
                       INCLUDE_DIRS "include"
                       REQUIRES core i2c_hub esp_driver_i2c esp_timer)
