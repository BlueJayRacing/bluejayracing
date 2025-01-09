idf_component_register( SRCS "sensorSetup.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_wifi
                            esp_system
                            esp_netif
                            esp_event
                            esp_timer 
                            nvs_flash
                            freertos
                            ad5626
                            ads1120
                            log
                            )