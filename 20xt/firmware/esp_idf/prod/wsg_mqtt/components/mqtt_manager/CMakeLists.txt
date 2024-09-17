idf_component_register(SRCS "mqtt_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                            driver
                            esp_system 
                            mqtt 
                            esp_wifi
                            log
                            )
