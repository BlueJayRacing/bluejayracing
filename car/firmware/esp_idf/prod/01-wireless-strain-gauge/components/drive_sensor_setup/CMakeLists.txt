idf_component_register( SRCS "driveSensorSetup.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system
                            esp_event
                            esp_timer 
                            freertos
                            ad5626
                            ads1120
                            log
                            )