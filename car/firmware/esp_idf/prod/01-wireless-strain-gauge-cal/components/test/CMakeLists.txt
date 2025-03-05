idf_component_register( SRCS "test.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system
                            esp_event
                            esp_timer 
                            freertos
                            sensorSetup
                            ads1120
                            ad5626
                            log
                            )