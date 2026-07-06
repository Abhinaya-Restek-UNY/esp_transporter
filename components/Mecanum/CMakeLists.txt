idf_component_register(SRCS "Mecanum.cpp"
    INCLUDE_DIRS "."
    REQUIRES bluepad32 pwmmanager IMU PID PresistentConfig
)
