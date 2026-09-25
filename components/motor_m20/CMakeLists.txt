idf_component_register(SRCS "deneyap_motor.c" "eksen.c"
                       INCLUDE_DIRS "include"
                       REQUIRES core i2c_hub esp_driver_i2c)
