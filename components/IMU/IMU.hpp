#pragma once
#include "MPU6050.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

static void IRAM_ATTR gpio_isr_handler(TaskHandle_t arg) {

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(arg, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
};

class IMU {
public:
  struct OffsetData {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
  };

  IMU(gpio_num_t SDA, gpio_num_t SCL, gpio_num_t interrupt);

  void set_offset_data(OffsetData offset);
  OffsetData get_offset();

  void reset_orientation();

  int8_t start(uint8_t calibration_loop = 0);
  int8_t start(OffsetData offset);

  void get_yaw(double *yaw, double *angular_yaw, uint64_t *timestamp);

private:
  SemaphoreHandle_t data_mutex;
  gpio_num_t SDA_pin;
  gpio_num_t SCL_pin;
  gpio_num_t interrupt_pin;

  uint16_t packetSize = 42;
  uint16_t fifoCount;
  uint8_t fifoBuffer[64];
  uint8_t mpuIntStatus;

  Quaternion raw_quat;
  VectorInt16 raw_gyro;

  OffsetData offset;

  uint64_t last_update = 0;

  i2c_master_bus_handle_t bus_handle;

  MPU6050 *mpu = nullptr;

  TaskHandle_t mpu_task_handle;

  void receive_packet();

  void setup_interrupt();
  static void mpu_task(IMU *mpu);

  int8_t init_hardware();
  int8_t finalize_start();
};
