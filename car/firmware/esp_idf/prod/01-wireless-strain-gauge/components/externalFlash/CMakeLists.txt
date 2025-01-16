idf_component_register( SRCS "externalFlash.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            log
                            dataFormat
                            )