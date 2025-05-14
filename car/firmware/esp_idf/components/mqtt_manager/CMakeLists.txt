idf_component_register( SRCS "mqttManager.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            mqtt
                            lock_guard
                            esp_wifi
                            esp_system
                            esp_netif
                            esp_event
                            esp_timer 
                            nvs_flash
                            freertos
                            log
                            )