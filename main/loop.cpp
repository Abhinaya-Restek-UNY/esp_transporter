#include "loop.hpp"
#include "btstack_run_loop.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

MainContext::MainContext()
    : imu(IMU_SDA, IMU_SCL, IMU_INT), device_mode("dev", DeviceMode::NORMAL),
      Servos(0, 50),
      Wheels(1, 20'000, BATTERY_VOLTAGE, BATTERY_VOLTAGE, DC_MOTOR_VOLTAGE),
      controll_config("ctrl", 0x00 |
                                  OrientationControllConfig::INCREMENT_ANGLE |
                                  GripperControllConfig::ANALOG_GRIPPER |
                                  DirectionControllConfig::ABSOLUTE_DIRECTION) {

  Servo *lifter1 = Servos.create_servo(GPIO_NUM_16, 180, 0.5, 2.5);
  Servo *lifter2 = Servos.create_servo(GPIO_NUM_2, 180, 0.5, 2.5);

  Servo *claw2 = Servos.create_servo(GPIO_NUM_4, 180, 0.5, 2.5);
  Servo *claw1 = Servos.create_servo(GPIO_NUM_15, 180, 0.5, 2.5);

  this->gripper[0] = new Gripper(claw1, lifter1, 90, "g1");
  this->gripper[1] = new Gripper(claw2, lifter2, 90, "g2");

  this->mt_br = Wheels.create_motor(BR_A, BR_B, BR_PWM);
  this->mt_bl = Wheels.create_motor(BL_A, BL_B, BL_PWM);
  this->mt_fr = Wheels.create_motor(FR_A, FR_B, FR_PWM);
  this->mt_fl = Wheels.create_motor(FL_A, FL_B, FL_PWM);

  esp_register_shutdown_handler(this->close);

  this->mecanum =
      new Mecanum(this->mt_fr, this->mt_fl, this->mt_br, this->mt_bl);
};

void MainContext::setup() {

  this->Wheels.start();
  this->Servos.start();

  gamepad.start();

  this->setup_adc();

  while (!this->gamepad.is_connected()) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (this->get_voltages() < 11.3) {
    printf("%f well\n", this->get_voltages());
    while (1) {
      this->gamepad.play_rumble();
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
};

bool MainContext::check_super_hotkey() {
  if (this->gamepad.is_pressed(Gamepad::ButtonCode::PS_BUTTON)) {
    DeviceMode prev = this->device_mode.value;
    if (this->gamepad.is_pressed(Gamepad::ButtonCode::R1)) {
      this->device_mode.value = DeviceMode::NORMAL;
    } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::R2)) {
      this->device_mode.value = DeviceMode::CALIBRATE_IMU;
    } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::L1)) {
      this->device_mode.value = DeviceMode::CALIBRATE_PID;
    } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::L2)) {
      this->device_mode.value = DeviceMode::SET_OPTION;
    } else if (this->gamepad.is_pressed(Gamepad::OPTIONS)) {
      esp_restart();
    } else if (this->gamepad.is_pressed(Gamepad::SHARE)) {
      this->device_mode.value = DeviceMode::CALIBRATE_DC_MOTOR;
    }
    if (prev != this->device_mode.value) {
      this->device_mode.save();
      return true;
    }
  }

  return false;
};

void MainContext::update_orientation() {
  imu.get_yaw(&this->yaw, &this->yaw_angular, NULL);
  if (fabs(imu.get_roll()) > M_PI_2) {
    this->rollin = true;
  } else {
    this->rollin = false;
  }
};

void MainContext::normal_mode_update_input_joy() {
  // Get current ljoy
  Gamepad::joy_data_t ljoy;
  gamepad.get_l_joy(&ljoy);

  // apply deadzone
  this->dead(&ljoy);

  // Check dpad also
  this->_dir_x =
      this->gamepad.get_dpad_x() ? this->gamepad.get_dpad_x() : (double)ljoy.x;
  this->_dir_y =
      this->gamepad.get_dpad_y() ? -this->gamepad.get_dpad_y() : (double)ljoy.y;

  double relative_yaw = this->yaw + M_PI_2;
  double _cos = cos(relative_yaw);
  double _sin = sin(relative_yaw);

  if (this->get_direction_config() ==
      DirectionControllConfig::ABSOLUTE_DIRECTION) {
    this->dir_x = (_dir_x * _cos + _dir_y * _sin);
    this->dir_y = (-_dir_x * _sin + _dir_y * _cos);
  }
}

void MainContext::close() {
  MainContext &instance = MainContext::getInstance();
  instance.mt_bl->set_direction(0);
  instance.mt_br->set_direction(0);
  instance.mt_fl->set_direction(0);
  instance.mt_fr->set_direction(0);
  btstack_run_loop_trigger_exit();
};

MainContext &MainContext::getInstance() {

  static MainContext isntance;
  return isntance;
};

bool MainContext::rjoy_set_absolute_angle_target() {
  Gamepad::joy_data_t rjoy;
  this->gamepad.get_r_joy(&rjoy);
  if (hypot(rjoy.x, rjoy.y) > RJOY_ABSOLUTE_ANGLE_DEADZONE) {
    this->yaw_target = this->yaw + wrap_rot(atan2(rjoy.y, rjoy.x));
    return true;
  }

  return false;
};

void MainContext::dead(Gamepad::joy_data_t *joy) {
  if (abs(joy->x) < JOYSTICK_DEADZONE) {
    joy->x = 0;
  }
  if (abs(joy->y) < JOYSTICK_DEADZONE) {
    joy->y = 0;
  }
}

void MainContext::set_controll_config(uint8_t mask, uint8_t value) {
  this->controll_config.value =
      (this->controll_config.value & (~mask)) | (value & mask);

  this->controll_config.save();
  this->gamepad.play_rumble();
};

void MainContext::set_orientation_config(OrientationControllConfig config) {
  this->set_controll_config(ControllConfigType::ORIENTATION_CONTROLL_CONFIG,
                            config);
};

void MainContext::set_direction_config(DirectionControllConfig config) {
  this->set_controll_config(ControllConfigType::DIRECTION_CONTROLL_CONFIG,
                            config);
};

void MainContext::set_gripper_config(GripperControllConfig config) {
  this->set_controll_config(ControllConfigType::GRIPPER_CONTROLL_CONFIG,
                            config);
};

OrientationControllConfig MainContext::get_orientation_config() {
  return (OrientationControllConfig)((this->controll_config.value) &
                                     ControllConfigType::
                                         ORIENTATION_CONTROLL_CONFIG);
};

DirectionControllConfig MainContext::get_direction_config() {
  return (
      DirectionControllConfig)((this->controll_config.value) &
                               ControllConfigType::DIRECTION_CONTROLL_CONFIG);
};

GripperControllConfig MainContext::get_gripper_config() {
  return (GripperControllConfig)((this->controll_config.value) &
                                 ControllConfigType::GRIPPER_CONTROLL_CONFIG);
};

bool MainContext::normal_mode_check_PS_hotkey() {

  if (this->gamepad.is_pressed(Gamepad::ButtonCode::PS_BUTTON)) {
    if (this->gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
      this->gamepad.zero_l_joy();
    } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::SQUARE)) {
      this->gamepad.play_rumble();
    }
    return true;
  }

  return false;
}

bool MainContext::normal_mode_check_speed_multiplier_hotkey() {
  if (this->gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
    this->speed_multiplier = 0.50;
  } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::SQUARE)) {
    this->speed_multiplier = 1.0;
  } else {
    return false;
  }

  return true;
}

bool MainContext::normal_mode_check_orientation_follow_direction() {
  if (this->get_direction_config() ==
      DirectionControllConfig::RELATIVE_DIRECTION) {
    return false;
  }

  if (this->gamepad.get_dpad_x() != 0 || this->gamepad.get_dpad_y() != 0) {

    double move_angle = atan2(gamepad.get_dpad_y(), gamepad.get_dpad_x());
    double error = wrap_rot(move_angle - yaw);

    if (fabs(error) <= (M_PI / 2.0)) {
      yaw_target = move_angle;
    } else {
      yaw_target = wrap_rot(move_angle + M_PI);
    }
    this->turn = pid.update(this->yaw_target, this->yaw, yaw_angular);
    return true;
  }

  return false;
};

bool MainContext::normal_mode_update_turn() {
  OrientationControllConfig orientation_config = this->get_orientation_config();
  static Gamepad::joy_data_t orientation_input;
  this->gamepad.get_r_joy(&orientation_input);
  this->dead(&orientation_input);

  switch (orientation_config) {

  case ABSOLUTE_ANGLE: {
    if (hypot(orientation_input.x, orientation_input.y) >
        RJOY_ABSOLUTE_ANGLE_DEADZONE) {
      this->yaw_target = (atan2(-orientation_input.y, -orientation_input.x));
    }

    this->turn =
        this->pid.update(this->yaw_target, this->yaw, this->yaw_angular);

    break;
  }

  case INCREMENT_ANGLE: {
    static bool is_turning = false;

    if (orientation_input.x != 0) {
      this->turn = -(double)orientation_input.x / 32768.0 * 0.8;
      is_turning = true;

    } else if (is_turning) {

      this->yaw_target = this->yaw;
      this->turn = 0;
      if (fabs(this->yaw_angular) < 0.3) {
        is_turning = false;
      }
    } else {
      this->turn =
          this->pid.update(this->yaw_target, this->yaw, this->yaw_angular);
    }

    break;
  }

  case RELATIVE_INCREMENT_ANGLE: {
    static bool is_turning = false;
    static double last_angle = 0;
    static double last_orientation = 0;
    if (hypot(orientation_input.x, orientation_input.y) > 16384) {

      double current_joystick_angle =
          atan2(-orientation_input.y, -orientation_input.x);

      if (is_turning) {
        double angle_delta = current_joystick_angle - last_angle;

        while (angle_delta > M_PI)
          angle_delta -= 2 * M_PI;
        while (angle_delta < -M_PI)
          angle_delta += 2 * M_PI;

        this->yaw_target =
            last_orientation + angle_delta * this->speed_multiplier;
      } else {
        last_angle = current_joystick_angle;
        last_orientation = this->yaw;
        is_turning = true;
      }
    } else {
      if (is_turning) {
        this->yaw_target = this->yaw;
        is_turning = false;
      }
    }

    this->turn =
        this->pid.update(this->yaw_target, this->yaw, this->yaw_angular);
    break;
  }
  default:
    break;
  }

  if (this->get_gripper_config() == GRIPPER_TO_TURN) {
    double tur = 0;

    if (this->grip_state == GRIP_STATE::NOT_GRIPPIN) {
      if (this->gamepad.get_brake() != 0 ||
          this->gamepad.is_pressed(Gamepad::ButtonCode::L1)) {
        this->grip_state = GRIP_STATE::LT;
      } else if (this->gamepad.get_throttle() != 0 ||
                 this->gamepad.is_pressed(Gamepad::ButtonCode::R1)) {
        this->grip_state = GRIP_STATE::RT;
      }
    } else if (this->gamepad.get_brake() == 0 &&
               this->gamepad.get_throttle() == 0 &&
               !this->gamepad.is_pressed(Gamepad::ButtonCode::L1) &&
               !this->gamepad.is_pressed(Gamepad::ButtonCode::R1)) {

      this->grip_state = GRIP_STATE::NOT_GRIPPIN;
    }

    if (this->grip_state == GRIP_STATE::LT) {
      tur = ((this->gamepad.is_pressed(Gamepad::ButtonCode::R1)) -
             this->gamepad.is_pressed(Gamepad::ButtonCode::R2));
    } else if (this->grip_state == GRIP_STATE::RT) {
      tur = ((this->gamepad.is_pressed(Gamepad::ButtonCode::L1)) -
             this->gamepad.is_pressed(Gamepad::ButtonCode::L2));
    }

    if (this->grip_state == GRIP_STATE::NOT_GRIPPIN) {

      this->gripper_speed_multiplier = 1.0;
    } else {
      this->gripper_speed_multiplier = 0.4;
    }

    static bool is_turning_wit_gripper = false;
    if (tur != 0) {

      this->turn = tur * this->speed_multiplier * 0.3;
      this->yaw_target = this->yaw;
      is_turning_wit_gripper = true;
    } else if (is_turning_wit_gripper) {

      this->turn = 0;
      this->yaw_target = this->yaw;
      if (fabs(this->yaw_angular) < 0.3) {
        is_turning_wit_gripper = false;
      }
    }
  }

  return true;
};

bool MainContext::normal_mode_update_mecanum() {
  // NOTE: CLAMP SO IT DOESNT CHASE STABLE SPEED BELOW SAFE VOLTAGE IN THE EVENT
  // OF VOLTAGE DROP
  float voltage =
      std::clamp(this->get_voltages(), 11.3f, (float)BATTERY_VOLTAGE);
  this->Wheels.update_voltage(voltage);
  // printf("Voltages %f\n", voltage);
  if (this->fallin_state == FALLIN_STATE::FORWARD) {
    this->gripper[0]->set_lifter(1024);
  } else if (this->fallin_state == FALLIN_STATE::BACKWARD) {
    this->gripper[1]->set_lifter(1024);
  }

  if (this->rollin) {

    this->mecanum->update(0, 0, 0);
  } else {

    double pitch = this->imu.get_pitch();
    if (pitch > M_PI_2 * 0.3) {
      this->fallin_state = FALLIN_STATE::FORWARD;

      // this->gripper[0].set_lifter(1024);
    } else if (pitch < -M_PI_2 * 0.3) {
      this->fallin_state = FALLIN_STATE::BACKWARD;
      // this->gripper[1].set_lifter(1024);
    } else if (fabs(pitch) < M_PI_2 * 0.1) {
      this->fallin_state = FALLIN_STATE::NOT_FALLIN;
    }

    this->mecanum->update(
        std::clamp((int)((double)this->dir_x * speed_multiplier *
                         gripper_speed_multiplier),
                   -32767, 32767),
        std::clamp((int)((double)this->dir_y * speed_multiplier *
                         gripper_speed_multiplier),
                   -32767, 32767),
        std::clamp((double)(this->turn),
                   -this->speed_multiplier * MAX_SPEED_MULTIPLIER,
                   this->speed_multiplier * MAX_SPEED_MULTIPLIER));
  }

  return true;
};

void MainContext::normal_mode_check_gripper_hotkey() {

  switch (this->get_gripper_config()) {

  case ANALOG_GRIPPER:
    this->gripper[0]->set_lifter(gamepad.get_throttle());
    this->gripper[0]->set_claw(gamepad.is_pressed(Gamepad::ButtonCode::R1));

    this->gripper[1]->set_lifter(gamepad.get_brake());
    this->gripper[1]->set_claw(gamepad.is_pressed(Gamepad::ButtonCode::L1));
    break;

  case GRIPPER_TO_TURN:
    if (this->grip_state != GRIP_STATE::LT) {
      this->gripper[0]->set_lifter(gamepad.get_throttle());
      this->gripper[0]->set_claw(gamepad.is_pressed(Gamepad::ButtonCode::R1));
    }
    if (this->grip_state != GRIP_STATE::RT) {
      this->gripper[1]->set_lifter(gamepad.get_brake());
      this->gripper[1]->set_claw(gamepad.is_pressed(Gamepad::ButtonCode::L1));
    }

    break;
  }
};

void MainContext::setup_adc() {
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(
      adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));

  adc_cali_line_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  esp_err_t cali_ret =
      adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
  if (cali_ret == ESP_OK) {
    calibrated = true;
  }

  int raw_value = 0;
  int voltage_mv = 0;

  // Read raw digital value (0 - 4095)
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw_value));

  if (calibrated) {
    ESP_ERROR_CHECK(
        adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv));
  }

  this->prev_voltage = voltage_mv * this->voltage_scale;
};

inline float applyLPF(float current_raw, float filtered_prev, float alpha) {
  return (alpha * current_raw) + ((1.0f - alpha) * filtered_prev);
}

float MainContext::get_voltages() {
  int raw_value = 0;
  int voltage_mv = 0;

  // Read raw digital value (0 - 4095)
  ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw_value));

  if (calibrated) {
    ESP_ERROR_CHECK(
        adc_cali_raw_to_voltage(cali_handle, raw_value, &voltage_mv));
  }

  this->prev_voltage =
      applyLPF(voltage_mv * this->voltage_scale, this->prev_voltage, 0.02);

  return this->prev_voltage;
}
