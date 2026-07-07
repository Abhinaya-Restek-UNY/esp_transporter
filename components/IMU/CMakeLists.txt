idf_component_register(SRCS "IMU.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c MPU6050 esp_PresistentConfig
)
