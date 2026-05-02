#include "IMU.hpp"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"

IMU::IMU(gpio_num_t SDA, gpio_num_t SCL, gpio_num_t interrupt)
    : SDA_pin(SDA), SCL_pin(SCL), interrupt_pin(interrupt) {
  i2c_master_bus_config_t bus_conf = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = SDA,
      .scl_io_num = SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true}};

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));
  this->data_mutex = xSemaphoreCreateMutex();
}

int8_t IMU::start(OffsetData offset) {
  this->offset = offset;

  if (init_hardware() != 0)
    return -1;

  mpu->setXAccelOffset(offset.accel_x);
  mpu->setYAccelOffset(offset.accel_y);
  mpu->setZAccelOffset(offset.accel_z);
  mpu->setXGyroOffset(offset.gyro_x);
  mpu->setYGyroOffset(offset.gyro_y);
  mpu->setZGyroOffset(offset.gyro_z);

  return finalize_start();
}

int8_t IMU::start(uint8_t calibration_loop) {
  if (init_hardware() != 0)
    return -2;

  if (calibration_loop > 0) {
    mpu->CalibrateAccel(calibration_loop);
    mpu->CalibrateGyro(calibration_loop);
  }

  int8_t status = finalize_start();

  this->offset.gyro_x = mpu->getXGyroOffset();
  this->offset.gyro_y = mpu->getYGyroOffset();
  this->offset.gyro_z = mpu->getZGyroOffset();
  this->offset.accel_x = mpu->getXAccelOffset();
  this->offset.accel_y = mpu->getYAccelOffset();
  this->offset.accel_z = mpu->getZAccelOffset();

  return status;
}

int8_t IMU::init_hardware() {
  if (this->mpu != nullptr) {
    return -1;
  }

  xTaskCreate((TaskFunction_t)mpu_task, "mpu_task", 2048, this, 10,
              &mpu_task_handle);

  this->mpu = new MPU6050(this->bus_handle);
  mpu->initialize();

  return mpu->dmpInitialize();
}

int8_t IMU::finalize_start() {
  mpu->setDMPEnabled(true);
  mpuIntStatus = mpu->getIntStatus();
  packetSize = mpu->dmpGetFIFOPacketSize();
  setup_interrupt();
  return 0; // Success
}

void IMU::setup_interrupt() {
  gpio_config_t io_conf{
      .pin_bit_mask = (1ULL << this->interrupt_pin),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en =
          GPIO_PULLUP_DISABLE, // MPU6050 INT is usually Push-Pull active high
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_POSEDGE, // Trigger on Rising Edge (0 -> 1)
  };

  gpio_config(&io_conf);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(this->interrupt_pin, (gpio_isr_t)gpio_isr_handler,
                       this->mpu_task_handle);
};

void IMU::set_offset_data(OffsetData offset) {
  this->offset = offset;

  mpu->setXGyroOffset(offset.gyro_x);
  mpu->setYGyroOffset(offset.gyro_y);
  mpu->setZGyroOffset(offset.gyro_z);

  mpu->setXAccelOffset(offset.accel_x);
  mpu->setYAccelOffset(offset.accel_y);
  mpu->setZAccelOffset(offset.accel_z);
};

IMU::OffsetData IMU::get_offset() { return this->offset; }

void IMU::mpu_task(IMU *context) {

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    context->receive_packet();
  }
}

void IMU::get_yaw(double *yaw, double *angular_yaw, uint64_t *timestamp) {
  if (xSemaphoreTake(this->data_mutex, portMAX_DELAY) == pdTRUE) {
    *yaw = atan2(2.0 * (this->raw_quat.w * this->raw_quat.z +
                        this->raw_quat.x * this->raw_quat.y),
                 1.0 - 2.0 * (this->raw_quat.y * this->raw_quat.y +
                              this->raw_quat.z * this->raw_quat.z));
    *angular_yaw = this->filtered_gyro_yaw / 16.4;
    *timestamp = this->last_update;
    xSemaphoreGive(this->data_mutex);
  }
};

void IMU::receive_packet() {
  mpu->dmpGetCurrentFIFOPacket(fifoBuffer);
  int64_t time_us = esp_timer_get_time();

  if (xSemaphoreTake(this->data_mutex, 100) == pdTRUE) {
    mpu->dmpGetQuaternion(&raw_quat, fifoBuffer);
    mpu->dmpGetGyro(&this->raw_gyro, fifoBuffer);
    this->filtered_gyro_yaw =
        IMU::lpf(this->raw_gyro.z, this->filtered_gyro_yaw, this->alpha);
    this->last_update = time_us;
    xSemaphoreGive(this->data_mutex);
  }
};

double IMU::lpf(double raw, double prev, double alpha) {
  return (alpha * raw) + (1.0 - alpha) * prev;
}
