idf_component_register(SRCS "Gripper.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_PWMManager Panic esp_timer esp_PresistentConfig
)
