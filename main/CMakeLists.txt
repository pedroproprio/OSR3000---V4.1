idf_component_register(SRCS "main.c"
                            "sensors/gps.c"
                            "sensors/bmp.c"
                            "sensors/icm.c"
                            "header.c"
                            "acquire.c"
                            "save_send.c"
                        INCLUDE_DIRS ".")