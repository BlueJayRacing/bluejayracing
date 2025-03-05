idf_component_register( SRCS "test.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            ads1120
                            esp_timer
                            log
                            )