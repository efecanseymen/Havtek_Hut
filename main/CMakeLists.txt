idf_component_register(SRCS "main.c" "rt_task.c" "cli.c"
                       INCLUDE_DIRS "."
                       REQUIRES core i2c_hub motor_m20 imu ahrs control webui
                                nvs_flash esp_timer)
