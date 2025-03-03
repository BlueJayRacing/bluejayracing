idf_component_register(SRCS "ads1120.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES 
                    driver
                    esp_system
                    log
                    freertos
                    )