#include "IMU.hpp"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"

IMU::IMU(gpio_num_t SDA, gpio_num_t SCL, gpio_num_t interrupt)
    : SDA_pin(SDA), SCL_pin(SCL), interrupt_pin(interrupt),
      offset("i", {0, 0, 0, 0, 0, 0}) {
  i2c_master_bus_config_t bus_conf = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = SDA,
      .scl_io_num = SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags = {.enable_internal_pullup = true}};

  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));

  // this->offset.remove();
}

int8_t IMU::start() {

  if (init_hardware() != 0)
    return -1;

  mpu->setXAccelOffset(this->offset.value.accel_x);
  mpu->setYAccelOffset(this->offset.value.accel_y);
  mpu->setZAccelOffset(this->offset.value.accel_z);
  mpu->setXGyroOffset(this->offset.value.gyro_x);
  mpu->setYGyroOffset(this->offset.value.gyro_y);
  mpu->setZGyroOffset(this->offset.value.gyro_z);

  return finalize_start();
}

int8_t IMU::start_and_calibrate(uint8_t calibration_loop) {
  if (init_hardware() != 0)
    return -2;

  if (calibration_loop > 0) {
    mpu->CalibrateAccel(calibration_loop);
    mpu->CalibrateGyro(calibration_loop);
  }

  int8_t status = finalize_start();

  this->offset.value.gyro_x = mpu->getXGyroOffset();
  this->offset.value.gyro_y = mpu->getYGyroOffset();
  this->offset.value.gyro_z = mpu->getZGyroOffset();
  this->offset.value.accel_x = mpu->getXAccelOffset();
  this->offset.value.accel_y = mpu->getYAccelOffset();
  this->offset.value.accel_z = mpu->getZAccelOffset();

  this->offset.save();

  return status;
}

int8_t IMU::init_hardware() {
  if (this->mpu != nullptr) {
    return -1;
  }

  this->mpu = new MPU6050(this->bus_handle);
  mpu->initialize();

  uint8_t ret = mpu->dmpInitialize();

  xTaskCreate((TaskFunction_t)mpu_task, "mpu_task", 8192, this, 10,
              &mpu_task_handle);

  return ret;
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

void IMU::mpu_task(IMU *context) {

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint16_t fifoCount = context->mpu->getFIFOCount();

    if (fifoCount > 500) {
      context->mpu->resetFIFO();
      continue;
    }

    if (!context->mpu->dmpGetCurrentFIFOPacket(context->fifoBuffer)) {
      continue;
    }

    int64_t time_us = esp_timer_get_time();

    context->mpu->dmpGetQuaternion(&context->raw_quat, context->fifoBuffer);
    context->mpu->dmpGetGravity(&context->gravity, &context->raw_quat);
    context->mpu->dmpGetYawPitchRoll(context->yawPitchRoll, &context->raw_quat,
                                     &context->gravity);
    context->mpu->dmpGetGyro(&context->raw_gyro, context->fifoBuffer);
    context->angular_yaw =
        IMU::lpf(context->raw_gyro.z, context->angular_yaw, context->alpha);
    context->yaw = context->yawPitchRoll[0];
    context->last_update = time_us;
  }
}

void IMU::get_yaw(double *yaw, double *angular_yaw, uint64_t *timestamp) {
  if (yaw != NULL) {
    double calc = (this->yaw - this->yaw_offset) + M_PI_2;

    while (calc > M_PI)
      calc -= 2.0 * M_PI;
    while (calc < -M_PI)
      calc += 2.0 * M_PI;

    *yaw = calc;
  }
  if (angular_yaw != NULL) {
    *angular_yaw = this->angular_yaw / 939.6508;
  }
  if (timestamp != NULL) {
    *timestamp = this->last_update;
  }
};

void IMU::offset_yaw() { this->yaw_offset = this->yaw; }
double IMU::lpf(double raw, double prev, double alpha) {
  return (alpha * raw) + (1.0 - alpha) * prev;
}
