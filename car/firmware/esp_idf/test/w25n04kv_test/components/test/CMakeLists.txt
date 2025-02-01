idf_component_register( SRCS "test.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            esp_timer
                            log
                            w25n04kv
                            )