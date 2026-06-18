idf_component_register(SRCS "PWMManager.cpp" "MotorManager.cpp" "ServoManager.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_mcpwm Panic esp_driver_gpio
)
