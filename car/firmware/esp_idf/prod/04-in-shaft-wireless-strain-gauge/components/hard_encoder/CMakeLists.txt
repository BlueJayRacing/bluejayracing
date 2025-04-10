idf_component_register( SRCS "hardEncoder.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            log
                            baja_nanopb
                            baja_proto
                            )