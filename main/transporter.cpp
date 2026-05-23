#include "Gripper.hpp"
#include "PID.hpp"
#include "driver/timer_types_legacy.h"
#include <algorithm>
#define PERMANENT_STEP_MULTIPLIER 0.01
#include "Gamepad.hpp"
#include "IMU.hpp"

#include "Mecanum.hpp"
#include "MotorHub.hpp"
#include "esp_task_wdt.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "nvs_flash.h"
#include "soc/gpio_num.h"
#include <cmath>
#include <stdio.h>

esp_err_t save_imu_to_nvs(IMU::OffsetData data) {
  nvs_handle_t my_handle;
  esp_err_t err;

  // 1. Buka NVS dengan namespace "storage"
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;

  // 2. Simpan struct sebagai Blob
  err = nvs_set_blob(my_handle, "imu_offsets", &data, sizeof(IMU::OffsetData));

  // 3. Commit (WAJIB! Kalau tidak, data tidak benar-benar tertulis)
  if (err == ESP_OK)
    err = nvs_commit(my_handle);

  // 4. Tutup handle
  nvs_close(my_handle);
  return err;
}

struct PID_Data {
  double Kp;
  double Ki;
  double Kd;
};

esp_err_t save_pid_to_nvs(PID_Data data) {
  nvs_handle_t my_handle;
  esp_err_t err;

  // 1. Buka NVS dengan namespace "storage"
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;

  // 2. Simpan struct sebagai Blob
  err = nvs_set_blob(my_handle, "pid", &data, sizeof(PID_Data));

  // 3. Commit (WAJIB! Kalau tidak, data tidak benar-benar tertulis)
  if (err == ESP_OK)
    err = nvs_commit(my_handle);

  // 4. Tutup handle
  nvs_close(my_handle);
  return err;
}

esp_err_t load_imu_from_nvs(PID_Data *out_data) {
  nvs_handle_t my_handle;
  esp_err_t err;
  size_t required_size = sizeof(PID_Data);

  err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
    return err;

  // Ambil data blob
  err = nvs_get_blob(my_handle, "pid", out_data, &required_size);

  nvs_close(my_handle);
  return err;
}

void delete_imu_nvs() {
  printf("Deleting imu calibraton..\n");
  nvs_handle_t my_handle;
  nvs_open("storage", NVS_READWRITE, &my_handle);

  nvs_erase_key(my_handle, "imu_offsets");
  nvs_commit(my_handle);
  nvs_close(my_handle);
};

esp_err_t load_imu_from_nvs(IMU::OffsetData *out_data) {
  nvs_handle_t my_handle;
  esp_err_t err;
  size_t required_size = sizeof(IMU::OffsetData);

  err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
    return err;

  // Ambil data blob
  err = nvs_get_blob(my_handle, "imu_offsets", out_data, &required_size);

  nvs_close(my_handle);
  return err;
}

void dead(Gamepad::joy_data_t *joy, uint8_t deadz) {

  if (abs(joy->x) < deadz) {
    joy->x = 0;
  }

  if (abs(joy->y) < deadz) {
    joy->y = 0;
  }
};

extern "C" void app_main(void) {

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  Gamepad gamepad;
  gamepad.start();

  printf("wort\n");
  while (!gamepad.is_connected()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  printf("woert\n");
  vTaskDelay(pdMS_TO_TICKS(200));

  gamepad.play_rumble(255, 200);
  vTaskDelay(pdMS_TO_TICKS(200));

  MotorHub mt_hub(1, 20'000, 7.4 / 5);

  mt_hub.start();

  // A1
  Motor *mt_br = mt_hub.create_motor(GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_23);
  // B1
  Motor *mt_bl = mt_hub.create_motor(GPIO_NUM_33, GPIO_NUM_5, GPIO_NUM_32);
  // A2
  Motor *mt_fr = mt_hub.create_motor(GPIO_NUM_14, GPIO_NUM_27, GPIO_NUM_13);
  // B2
  Motor *mt_fl = mt_hub.create_motor(GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_17);

  IMU::OffsetData calib = {};

  IMU imu(GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_34);
  if (load_imu_from_nvs(&calib) == ESP_ERR_NVS_NOT_FOUND) {
    printf("Start calibrating...\n");
    imu.start(6);
    save_imu_to_nvs(imu.get_offset());
    vTaskDelay(pdMS_TO_TICKS(500));
  } else {

    printf("Load from previos calibration...\n");
    imu.start(calib);
  }

  Mecanum mecanum(mt_fr, mt_fl, mt_br, mt_bl);

  Gamepad::joy_data_t orientation;
  Gamepad::joy_data_t direction;

  double speed_multiplier = 1.0;

  MotorHub servos(0, 50);
  servos.start();

  Servo *lifter1 = servos.create_servo(GPIO_NUM_16, 180, 0.5, 2.5);
  Servo *lifter2 = servos.create_servo(GPIO_NUM_2, 180, 0.5, 2.5);

  Servo *claw1 = servos.create_servo(GPIO_NUM_4, 180, 0.5, 2.5);
  Servo *claw2 = servos.create_servo(GPIO_NUM_15, 180, 0.5, 2.5);
  Gripper griper[2] = {Gripper(claw1, lifter1, 3, 90, 0),
                       Gripper(claw2, lifter2, 3, 90, 0)};
  // double permanent_multiplier = 1.0;

  double yaw = 0;
  double imu_cos = 0;
  double imu_sin = 0;
  double angular = 0;

  uint16_t joy_deadzone = 320.0;

  double yaw_offset = 0;
  uint8_t servo_mode = 0;
  PID_Data pid_conf = {0, 0, 0};

  if (load_imu_from_nvs(&pid_conf) == ESP_ERR_NVS_NOT_FOUND) {
  }
  PID myYawPID(1.7556, // Kp
               0.000000, 0.00,
               10.0, // delta_t_ms (Assuming a 100Hz control loop)
               -1.0, // limMin
               1.0,  // limMax
               -0.3, // limMinInt
               0.3   // limMaxInt
  );

  // delete_imu_nvs();

  double setpoint = 0.0;
  while (1) {

    gamepad.get_l_joy(&direction);
    gamepad.get_r_joy(&orientation);

    dead(&direction, joy_deadzone);
    dead(&orientation, joy_deadzone);

    if (gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
      pid_conf.Kp += 0.001;
      myYawPID.set_param(pid_conf.Kp, pid_conf.Ki, pid_conf.Kd);
    } else if (gamepad.is_pressed(Gamepad::ButtonCode::CROSS)) {
      pid_conf.Kp -= 0.001;
      myYawPID.set_param(pid_conf.Kp, pid_conf.Ki, pid_conf.Kd);
    }
    //
    if (gamepad.is_pressed(Gamepad::ButtonCode::R1)) {
      pid_conf.Kd += 0.00001;
      myYawPID.set_param(pid_conf.Kp, pid_conf.Ki, pid_conf.Kd);
    } else if (gamepad.is_pressed(Gamepad::ButtonCode::R2)) {
      pid_conf.Kd -= 0.00001;
      myYawPID.set_param(pid_conf.Kp, pid_conf.Ki, pid_conf.Kd);
    }

    if (gamepad.is_pressed(Gamepad::ButtonCode::L1)) {
      pid_conf.Ki += 0.00001;
      myYawPID.set_param(pid_conf.Kp, pid_conf.Ki, pid_conf.Kd);
    } else if (gamepad.is_pressed(Gamepad::ButtonCode::L2)) {
      pid_conf.Ki -= 0.00001;
      myYawPID.set_param(pid_conf.Kp, pid_conf.Ki, pid_conf.Kd);
    }

    if (gamepad.is_pressed(Gamepad::SQUARE)) {
      save_pid_to_nvs(pid_conf);
    }

    if (gamepad.is_pressed(Gamepad::ButtonCode::PS_BUTTON)) {
      if (gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
        gamepad.zero_r_joy();
        gamepad.zero_l_joy();
      } else if (gamepad.is_pressed(Gamepad::ButtonCode::CIRCLE)) {
        yaw_offset = yaw;
        setpoint = 0;
      } else if (gamepad.is_pressed(Gamepad::ButtonCode::SQUARE)) {
        gamepad.play_rumble(200, 100);
      }
    } else {
      // if (gamepad.is_pressed(Gamepad::ButtonCode::SHARE)) {
      //   servo_mode = 1;
      // } else if (gamepad.is_pressed(Gamepad::ButtonCode::OPTIONS)) {
      //   servo_mode = 0;
      // }
      //
      // if (gamepad.is_pressed(Gamepad::ButtonCode::SQUARE)) {
      //   griper[servo_mode].open();
      //
      // } else if (gamepad.is_pressed(Gamepad::ButtonCode::CIRCLE)) {
      //   griper[servo_mode].close();
      // }
      //
      // if (gamepad.is_pressed(Gamepad::ButtonCode::CROSS)) {
      //
      //   griper[servo_mode].drop();
      // } else if (gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
      //   griper[servo_mode].lift();
      // }

      if (gamepad.is_pressed(Gamepad::ButtonCode::R1)) {
        speed_multiplier = 0.50;
      } else if (gamepad.is_pressed(Gamepad::ButtonCode::R2)) {
        speed_multiplier = 0.30;
      } else if (gamepad.is_pressed(Gamepad::ButtonCode::L1)) {
        speed_multiplier = 1.0;
      } else if (gamepad.is_pressed(Gamepad::ButtonCode::L2)) {
        speed_multiplier = 0.75;
      }
    }

    double input_x =
        gamepad.get_dpad_x() ? gamepad.get_dpad_x() : (double)direction.x;

    double input_y =
        gamepad.get_dpad_y() ? gamepad.get_dpad_y() : -(double)direction.y;

    imu.get_yaw(&yaw, &angular, NULL);

    double angle = -(yaw - yaw_offset);

    imu_cos = cos(angle);
    imu_sin = sin(angle);

    double rotated_x = input_x * imu_cos + input_y * imu_sin;
    double rotated_y = -input_x * imu_sin + input_y * imu_cos;

    if (hypot(orientation.x, orientation.y) > 16384) {
      setpoint = atan2(-orientation.x, -orientation.y);
    }

    mecanum.update(
        std::clamp((int)(rotated_x * speed_multiplier), -32767, 32767),
        std::clamp((int)(rotated_y * speed_multiplier), -32767, 32767),
        -myYawPID.update(setpoint, angle, -angular));

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
