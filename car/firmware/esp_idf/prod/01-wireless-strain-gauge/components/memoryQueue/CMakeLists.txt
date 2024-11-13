idf_component_register( SRCS "memoryQueue.cpp" "memoryBlock.cpp"
                        INCLUDE_DIRS "include"
                        REQUIRES 
                            driver
                            esp_system 
                            log
                            )