idf_component_register(SRCS "Gripper.cpp"
    INCLUDE_DIRS "."
    REQUIRES MotorHub Panic esp_timer PresistentConfig
)
