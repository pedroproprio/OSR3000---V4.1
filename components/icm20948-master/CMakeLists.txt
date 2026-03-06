idf_component_register(SRCS "icm20948.c" "icm20948_i2c.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)
