idf_component_register(SRCS "Mecanum.cpp"
    INCLUDE_DIRS "."
    REQUIRES bluepad32 esp_PWMManager IMU PID esp_PresistentConfig
)
