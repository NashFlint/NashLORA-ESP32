idf_component_register(SRCS "NashLORA.cpp"
                    REQUIRES driver freertos esp_system esp_timer
                    INCLUDE_DIRS "include")