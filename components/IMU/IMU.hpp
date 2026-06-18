#pragma once
#include "MPU6050.h"
#include "PresistentConfig.hpp"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <atomic>

/**
 * @brief Interrupt Service Routine (ISR) handler for the IMU's INT pin.
 * Signals the dedicated FreeRTOS task to wake up and fetch the latest DMP
 * packet.
 * * @param arg A pointer to the TaskHandle_t of the task waiting for the
 * notification.
 */
static void IRAM_ATTR gpio_isr_handler(TaskHandle_t arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(arg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
};

/**
 * @brief ESP-IDF wrapper class for the MPU6050 IMU.
 * Handles I2C initialization, DMP configuration, hardware interrupts,
 * and thread-safe data acquisition using FreeRTOS.
 */
class IMU {
public:
  /**
   * @brief Structure to hold hardware calibration offsets.
   */
  struct OffsetData {
    int16_t gyro_x;  ///< X-axis gyroscope offset
    int16_t gyro_y;  ///< Y-axis gyroscope offset
    int16_t gyro_z;  ///< Z-axis gyroscope offset
    int16_t accel_x; ///< X-axis accelerometer offset
    int16_t accel_y; ///< Y-axis accelerometer offset
    int16_t accel_z; ///< Z-axis accelerometer offset
  };

  /**
   * @brief Constructs the IMU object and configures the I2C master bus.
   * * @param SDA GPIO pin used for I2C Serial Data.
   * @param SCL GPIO pin used for I2C Serial Clock.
   * @param interrupt GPIO pin connected to the MPU6050 hardware interrupt pin.
   */
  IMU(gpio_num_t SDA, gpio_num_t SCL, gpio_num_t interrupt);

  int8_t start();

  /**
   * @brief Thread-safe method to retrieve the latest yaw and angular velocity
   * data.
   * * @param yaw Pointer to store the calculated Euler yaw angle (in radians).
   * @param angular_yaw Pointer to store the raw Z-axis gyroscope velocity.
   * @param timestamp Pointer to store the timestamp (in microseconds) of this
   * reading.
   */
  void get_yaw(double *yaw, double *angular_yaw, uint64_t *timestamp);
  double get_pitch();
  double get_roll();

  void offset_yaw();

  void offset_pitchroll();
  int8_t start_and_calibrate(uint8_t calibration_loop);

private:
  double alpha = 0.38;
  // --- Thread Safety ---

  // --- Hardware Pins ---
  gpio_num_t SDA_pin;       ///< I2C SDA pin
  gpio_num_t SCL_pin;       ///< I2C SCL pin
  gpio_num_t interrupt_pin; ///< MPU6050 external interrupt pin
                            ///

  // --- MPU Internal State Variables ---
  uint16_t packetSize = 42; ///< Expected DMP packet size
  uint16_t fifoCount = 0;   ///< Current bytes in the MPU FIFO buffer
  uint8_t fifoBuffer[64];   ///< Buffer to hold fetched DMP packet
  uint8_t mpuIntStatus;     ///< Holds actual interrupt status byte from MPU

  // --- Data Storage ---
  Quaternion raw_quat; ///< Latest raw quaternion fetched from the DMP
  float yawPitchRoll[3] = {0, 0, 0};
  std::atomic<double> yaw = 0;
  std::atomic<double> angular_yaw = 0;
  std::atomic<double> pitch = 0;
  std::atomic<double> roll = 0;
  VectorFloat gravity = {0, 0, 0};
  VectorInt16 raw_gyro = {
      0, 0, 0}; ///< Latest raw gyroscope vector fetched from the DMP
  std::atomic<uint64_t> last_update =
      0; ///< Timestamp (us) of the last successful data fetch

  // --- Handles & Pointers ---
  i2c_master_bus_handle_t bus_handle; ///< ESP-IDF I2C bus handle
  MPU6050 *mpu = nullptr; ///< Pointer to the MPU6050 driver instance
  TaskHandle_t
      mpu_task_handle; ///< FreeRTOS handle for the data processing task

  // --- Internal Helper Methods ---

  /**
   * @brief Configures the ESP32 GPIO interrupt and attaches the ISR handler.
   */
  void setup_interrupt();

  /**
   * @brief FreeRTOS task function that waits for an ISR notification to process
   * data.
   * @param mpu Pointer to the IMU class context.
   */
  static void mpu_task(IMU *mpu);

  /**
   * @brief Instantiates the MPU object and spins up the background FreeRTOS
   * task.
   * @return int8_t 0 on success, negative error code on failure.
   */
  int8_t init_hardware();

  /**
   * @brief Finalizes the startup sequence, enabling the DMP and attaching the
   * interrupt.
   * @return int8_t 0 on success.
   */
  int8_t finalize_start();

  static double lpf(double raw, double prev, double alpha);

  PresistentConfig<OffsetData> offset;
  double yaw_offset = 0;
  double pitch_offset = 0;
  double roll_offset = 0;
};
