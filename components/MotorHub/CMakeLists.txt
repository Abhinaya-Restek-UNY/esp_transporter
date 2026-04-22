idf_component_register(SRCS "MotorHub.cpp" "Motor.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_mcpwm Panic esp_driver_gpio
)
