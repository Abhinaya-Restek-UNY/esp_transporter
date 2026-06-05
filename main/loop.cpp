#include "loop.hpp"
#include "esp_system.h"
#include <algorithm>
#include <cmath>

MainContext::MainContext()
    : imu(IMU_SDA, IMU_SCL, IMU_INT), device_mode("dev", DeviceMode::NORMAL),
      Servos(0, 50), Wheels(1, 20'000, BATTERY_VOLTAGE / DC_MOTOR_VOLTAGE),
      controll_config("ctrl", 0x00 |
                                  OrientationControllConfig::INCREMENT_ANGLE |
                                  GripperControllConfig::ANALOG_GRIPPER |
                                  DirectionControllConfig::ABSOLUTE_DIRECTION) {

  Servo *lifter1 = Servos.create_servo(GPIO_NUM_16, 180, 0.5, 2.5);
  Servo *lifter2 = Servos.create_servo(GPIO_NUM_2, 180, 0.5, 2.5);

  Servo *claw2 = Servos.create_servo(GPIO_NUM_4, 180, 0.5, 2.5);
  Servo *claw1 = Servos.create_servo(GPIO_NUM_15, 180, 0.5, 2.5);

  this->gripper[0] = Gripper(claw1, lifter1, pdMS_TO_TICKS(500));
  this->gripper[1] = Gripper(claw2, lifter2, pdMS_TO_TICKS(500));

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

  while (!this->gamepad.is_connected()) {
    vTaskDelay(pdMS_TO_TICKS(100));
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
    }

    if (prev != this->device_mode.value) {
      this->device_mode.save();
      return true;
    }
  }

  return false;
};

void MainContext::update_yaw() {
  imu.get_yaw(&this->yaw, &this->yaw_angular, NULL);
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
    } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::CIRCLE)) {
      this->imu.offset_yaw();
      this->imu.get_yaw(&this->yaw, &this->yaw_angular, NULL);
      this->yaw_target = this->yaw;
    } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::SQUARE)) {
      this->gamepad.play_rumble(200, 100);
    }
    return true;
  }

  return false;
}

bool MainContext::normal_mode_check_speed_multiplier_hotkey() {
  if (this->gamepad.is_pressed(Gamepad::ButtonCode::CIRCLE)) {
    this->speed_multiplier = 0.30;
  } else if (this->gamepad.is_pressed(Gamepad::ButtonCode::TRIANGLE)) {
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

  if (hypot(this->_dir_x, this->_dir_y) > 16384 &&
      this->gamepad.is_pressed(Gamepad::ButtonCode::CROSS)) {

    double move_angle = atan2(this->_dir_y, this->_dir_x);
    double error = wrap_rot(move_angle - yaw);

    if (fabs(error) <= (M_PI / 2.0)) {
      yaw_target = move_angle;
    } else {
      yaw_target = wrap_rot(move_angle + M_PI);
    }
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

    return true;
    break;
  }

  case INCREMENT_ANGLE: {
    static bool is_turning = false;

    if (orientation_input.x != 0) {
      this->turn = -(double)orientation_input.x / 32768.0;
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

    return true;
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

        this->yaw_target = last_orientation + angle_delta;
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

    // 2. Update PID with the safely bounded target
    this->turn =
        this->pid.update(this->yaw_target, this->yaw, this->yaw_angular);

    return true;
  }
  }

  return false;
};

bool MainContext::normal_mode_update_mecanum() {
  this->mecanum->update(
      std::clamp((int)(this->dir_x * this->speed_multiplier), -32767, 32767),
      std::clamp((int)(this->dir_y * this->speed_multiplier), -32767, 32767),
      std::clamp((double)(this->turn), -this->speed_multiplier,
                 this->speed_multiplier));
  return true;
};

void MainContext::normal_mode_check_gripper_hotkey() {
  switch (this->get_gripper_config()) {

  case ANALOG_GRIPPER:
    this->gripper[0].set_lifter(MAX_THROTTLE_BRAKE - gamepad.get_throttle());
    this->gripper[0].set_claw(gamepad.is_pressed(Gamepad::ButtonCode::R1));

    this->gripper[1].set_lifter(MAX_THROTTLE_BRAKE - gamepad.get_brake());
    this->gripper[1].set_claw(gamepad.is_pressed(Gamepad::ButtonCode::L1));
    break;

  case TOGGLE_GRIPPER:

  case NORMALLY_CLOSED_GRIPPER:

    this->gripper[0].set_claw(
        this->gamepad.is_pressed(Gamepad::ButtonCode::R1));
    this->gripper[0].set_lifter(
        this->gamepad.is_pressed(Gamepad::ButtonCode::R2) ? 0 : 1024);

    this->gripper[1].set_claw(
        this->gamepad.is_pressed(Gamepad::ButtonCode::L1));
    this->gripper[1].set_lifter(
        this->gamepad.is_pressed(Gamepad::ButtonCode::L2) ? 0 : 1024);
    break;
  }
};
