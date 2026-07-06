idf_component_register(SRCS "Gripper.cpp"
    INCLUDE_DIRS "."
    REQUIRES pwmmanager Panic esp_timer PresistentConfig
)
