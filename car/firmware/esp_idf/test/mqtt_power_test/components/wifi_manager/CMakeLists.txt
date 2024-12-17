idf_component_register(SRCS "wifi_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES    driver
                                esp_system
                                esp_wifi 
                                nvs_flash
                                esp_netif
                                esp_event
                                esp_timer
                                log
                                )
