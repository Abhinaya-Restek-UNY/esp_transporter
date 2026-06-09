#pragma once
#define HORIZONTAL_ANGLE_INCREMENT
#include "nvs_flash.h"
#include <string> // #define CALIBRATE_PID

enum class DeviceMode : uint8_t {
  CALIBRATE_PID = 1,
  CALIBRATE_IMU = 2,
  SET_OPTION = 3,
  NORMAL = 4,
  CALIBRATE_DC_MOTOR = 5
};

enum ControllConfigType {
  GRIPPER_CONTROLL_CONFIG = 0b111'00'000,
  ORIENTATION_CONTROLL_CONFIG = 0b000'00'111,
  DIRECTION_CONTROLL_CONFIG = 0b000'11'000
};

enum OrientationControllConfig {
  ABSOLUTE_ANGLE = 0b000'0'0001,
  INCREMENT_ANGLE = 0b000'0'0010,
  RELATIVE_INCREMENT_ANGLE = 0b000'00'100,
};

enum DirectionControllConfig {
  ABSOLUTE_DIRECTION = 0b000'01'000,
  RELATIVE_DIRECTION = 0b000'10'000,
};

enum GripperControllConfig {
  ANALOG_GRIPPER = 0b001'00'000,
  GRIPPER_TO_TURN = 0b010'00'000,
};

#define BR_B GPIO_NUM_19
#define BR_A GPIO_NUM_18
#define BR_PWM GPIO_NUM_23

#define BL_B GPIO_NUM_33
#define BL_A GPIO_NUM_5
#define BL_PWM GPIO_NUM_32

#define FR_B GPIO_NUM_14
#define FR_A GPIO_NUM_27
#define FR_PWM GPIO_NUM_13

#define FL_B GPIO_NUM_26
#define FL_A GPIO_NUM_25
#define FL_PWM GPIO_NUM_17

#define IMU_SCL GPIO_NUM_22
#define IMU_SDA GPIO_NUM_21
#define IMU_INT GPIO_NUM_34

// NOTE: Value derived from 32768 * 0.1
#define JOYSTICK_DEADZONE 512

#define IMU_CALIBRATION_LOOP 6

// NOTE: value derived from hypot(32768, 32768) * 0.5
#define RJOY_ABSOLUTE_ANGLE_DEADZONE 23170

#define BATTERY_VOLTAGE 12.0
#define DC_MOTOR_VOLTAGE 5.0

#define MAX_SPEED_MULTIPLIER 1
