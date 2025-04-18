idf_component_register( SRCS "test.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            log
                            mqtt_manager
                            config
                            cal_sensor_setup
                            drive_sensor_setup
                            external_flash
                            esp_timer
                            ads1120
                            ad5689r
                            baja_nanopb
                            baja_proto
                            hard_encoder
                            )