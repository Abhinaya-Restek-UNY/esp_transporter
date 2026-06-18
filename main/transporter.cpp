
#include "Gamepad.hpp"
#include "config.hpp"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "loop.hpp"
#include <cmath>
#include <stdio.h>

extern "C" void app_main(void) {
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  MainContext &ctx = MainContext::getInstance();
  ctx.setup();

  vTaskDelay(pdMS_TO_TICKS(100));

  ctx.gamepad.play_rumble();

  vTaskDelay(pdMS_TO_TICKS(100));
  if (ctx.device_mode.value == DeviceMode::CALIBRATE_IMU) {

    ctx.imu.start_and_calibrate(IMU_CALIBRATION_LOOP);
    ctx.gamepad.play_rumble();

    ctx.device_mode.value = DeviceMode::NORMAL;
    ctx.device_mode.save();

    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();

  } else {
    ctx.imu.start();
  }

  ctx.set_direction_config(DirectionControllConfig::ABSOLUTE_DIRECTION);

  while (1) {

    ctx.gamepad.play_rumble();
    printf("Mode: %d\n", (uint8_t)ctx.device_mode.value);
    if (ctx.device_mode.value == DeviceMode::CALIBRATE_IMU) {
      break;
    } else if (ctx.device_mode.value == DeviceMode::CALIBRATE_DC_MOTOR) {
      TickType_t lastwaketime = xTaskGetTickCount();

      const TickType_t frequency = pdMS_TO_TICKS(10);

      while (!ctx.check_super_hotkey()) {

        ctx.update_orientation();
        if (!ctx.normal_mode_check_PS_hotkey()) {
        }

        ctx.normal_mode_update_input_joy();

        // if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::CIRCLE)) {
        //   ctx.mecanum->correction.value.vertical += 0.0001;
        //
        // } else if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::SQUARE)) {
        //   ctx.mecanum->correction.value.vertical -= 0.0001;
        // }
        //
        // if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
        //   ctx.mecanum->correction.value.vertical += 0.0001;
        //
        // } else if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::CROSS)) {
        //   ctx.mecanum->correction.value.vertical -= 0.0001;
        // }
        // if (ctx.gamepad.is_just_pressed(Gamepad::OPTIONS)) {
        //   ctx.mecanum->correction.save();
        //
        // } else if (ctx.gamepad.is_just_pressed(Gamepad::ButtonCode::SHARE)) {
        //   ctx.mecanum->correction.value = {0, 0};
        // }
        //
        ctx.normal_mode_update_turn();

        ctx.normal_mode_update_mecanum();
      }

      vTaskDelayUntil(&lastwaketime, frequency);
    } else if (ctx.device_mode.value == DeviceMode::CALIBRATE_PID) {

      TickType_t lastwaketime = xTaskGetTickCount();
      const TickType_t frequency = pdMS_TO_TICKS(10);

      while (!ctx.check_super_hotkey()) {

        if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
          ctx.pid.config->Kp += 0.001;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        } else if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::CROSS)) {
          ctx.pid.config->Kp -= 0.001;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        }
        //
        if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::R1)) {
          ctx.pid.config->Kd += 0.0001;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        } else if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::R2)) {
          ctx.pid.config->Kd -= 0.0001;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        }

        if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::L1)) {
          ctx.pid.config->Ki += 0.00001;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        } else if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::L2)) {
          ctx.pid.config->Ki -= 0.00001;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        }

        if (ctx.gamepad.is_just_pressed(Gamepad::SQUARE)) {
          ctx.pid.save_config();
        } else if (ctx.gamepad.is_just_pressed(Gamepad::CIRCLE)) {
          ctx.pid.config->Kd = 0;
          ctx.pid.config->Ki = 0;
          ctx.pid.config->Kp = 0;
          printf("kp: %f, kd: %f, ki: %f\n", ctx.pid.config->Kp,
                 ctx.pid.config->Kd, ctx.pid.config->Ki);
        }

        ctx.update_orientation();

        ctx.normal_mode_update_input_joy();

        if (!ctx.normal_mode_check_orientation_follow_direction()) {
          ctx.normal_mode_update_turn();
        }

        ctx.normal_mode_update_mecanum();

        vTaskDelayUntil(&lastwaketime, frequency);
      }
    } else if (ctx.device_mode.value == DeviceMode::SET_OPTION) {

      TickType_t lastwaketime = xTaskGetTickCount();
      const TickType_t frequency = pdMS_TO_TICKS(10);

      while (!ctx.check_super_hotkey()) {
        if (ctx.gamepad.is_just_pressed(Gamepad::ButtonCode::CROSS)) {
          ctx.set_orientation_config(OrientationControllConfig::ABSOLUTE_ANGLE);
        } else if (ctx.gamepad.is_just_pressed(Gamepad::ButtonCode::SQUARE)) {
          ctx.set_orientation_config(
              OrientationControllConfig::RELATIVE_INCREMENT_ANGLE);
        } else if (ctx.gamepad.is_just_pressed(Gamepad::ButtonCode::TRIANGLE)) {
          ctx.set_orientation_config(
              OrientationControllConfig::INCREMENT_ANGLE);
        }

        if (ctx.gamepad.is_just_pressed(Gamepad::L1)) {
          ctx.set_gripper_config(GripperControllConfig::ANALOG_GRIPPER);
        } else if (ctx.gamepad.is_just_pressed(Gamepad::L2)) {
          ctx.set_gripper_config(GripperControllConfig::GRIPPER_TO_TURN);
        }

        if (ctx.gamepad.is_just_pressed(Gamepad::R1)) {
          ctx.set_direction_config(DirectionControllConfig::RELATIVE_DIRECTION);
        } else if (ctx.gamepad.is_just_pressed(Gamepad::R2)) {
          ctx.set_direction_config(DirectionControllConfig::ABSOLUTE_DIRECTION);
        }

        vTaskDelayUntil(&lastwaketime, frequency);
      }
    } else if (ctx.device_mode.value == DeviceMode::NORMAL) {

      TickType_t lastwaketime = xTaskGetTickCount();

      const TickType_t frequency = pdMS_TO_TICKS(10);

      while (!ctx.check_super_hotkey()) {

        if (ctx.get_direction_config() ==
            DirectionControllConfig::ABSOLUTE_DIRECTION) {
          ctx.update_orientation();
        }

        if (!ctx.normal_mode_check_PS_hotkey()) {
          ctx.normal_mode_check_gripper_hotkey();
          ctx.normal_mode_check_speed_multiplier_hotkey();
        }

        if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::SHARE) &&
            ctx.gamepad.is_pressed(Gamepad::ButtonCode::OPTIONS) &&
            ctx.gamepad.is_pressed(Gamepad::ButtonCode::L1) &&
            ctx.gamepad.is_pressed(Gamepad::ButtonCode::R1)) {
          ctx.gamepad.play_rumble();
          ctx.gamepad.unlock();
          break;
        } else if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::SHARE) &&
                   ctx.gamepad.is_pressed(Gamepad::ButtonCode::OPTIONS)) {
          ctx.gamepad.play_rumble();
          ctx.gamepad.lock();
        }

        if (ctx.gamepad.is_pressed(Gamepad::ButtonCode::CROSS)) {
          ctx.imu.offset_yaw();
          ctx.imu.get_yaw(&ctx.yaw, &ctx.yaw_angular, NULL);
          ctx.yaw_target = ctx.yaw;
        }

        ctx.normal_mode_update_input_joy();

        if (!ctx.normal_mode_check_orientation_follow_direction()) {
          ctx.normal_mode_update_turn();
        }

        ctx.normal_mode_update_mecanum();

        vTaskDelayUntil(&lastwaketime, frequency);
      }
    }
  }

  esp_restart();
}
