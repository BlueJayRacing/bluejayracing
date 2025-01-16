idf_component_register( SRCS "test.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            log
                            memoryQueue
                            mqttManager
                            externalFlash
                            esp_timer
                            ads1120
                            ad5626
                            )