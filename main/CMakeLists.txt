idf_component_register(SRCS main.c
                            sensors/adc.c
                            sensors/bmp.c
                            sensors/gps.c
                            sensors/icm.c
                            acquire.c
                            header.c
                            save_send.c
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES
                            esp_adc
                            esp_timer
                            esp_driver_i2c
                            esp_driver_uart
                            nvs_flash
                            sdmmc
                            fatfs
                            esp_bmp390
                            esp_littlefs-master
                            icm20948-master
                            minmea-master
                            vqf-main
                            eskf)