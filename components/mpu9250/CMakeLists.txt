idf_component_register(SRCS "mpu9250.c" "imu.c"
                        INCLUDE_DIRS include
                        REQUIRES bus_config)
