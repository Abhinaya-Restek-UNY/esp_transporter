#include "Gamepad.hpp"
#include "Gripper.hpp"
#include "IMU.hpp"
#include "Mecanum.hpp"
#include "MotorManager.hpp"
#include "PID.hpp"
#include "PresistentConfig.hpp"
#include "ServoManager.hpp"
#include "config.hpp"
#include "esp_adc/adc_oneshot.h"

inline double wrap_rot(double yaw) {

  while (yaw > M_PI)
    yaw -= 2.0 * M_PI;
  while (yaw < -M_PI)
    yaw += 2.0 * M_PI;
  return yaw;
}
class MainContext {

public:
  MainContext(const MainContext &) = delete;
  MainContext &operator=(const MainContext &) = delete;

  static MainContext &getInstance();
  static void close();

  void setup();
  Mecanum *mecanum;
  Gripper *gripper[2];

  // NOTE: Face forward at start.
  double yaw_target = M_PI_2;

  IMU imu;

  PID pid;

  Gamepad gamepad;

  PresistentConfig<DeviceMode> device_mode;

  double dir_x = 0;
  double dir_y = 0;
  double yaw = 0;
  double speed_multiplier = 1.0;
  double yaw_angular = 0;

  bool check_super_hotkey();

  void update_orientation();
  void normal_mode_update_input_joy();

  void normal_mode_check_gripper_hotkey();
  bool normal_mode_check_speed_multiplier_hotkey();
  bool normal_mode_check_PS_hotkey();
  bool normal_mode_check_orientation_follow_direction();
  bool normal_mode_update_turn();
  bool normal_mode_update_mecanum();

  bool rjoy_set_absolute_angle_target();

  void dead(Gamepad::joy_data_t *joy);

  void set_orientation_config(OrientationControllConfig config);
  void set_direction_config(DirectionControllConfig config);
  void set_gripper_config(GripperControllConfig config);

  OrientationControllConfig get_orientation_config();
  DirectionControllConfig get_direction_config();
  GripperControllConfig get_gripper_config();

private:
  enum GRIP_STATE { RT, LT, NOT_GRIPPIN };
  enum FALLIN_STATE { NOT_FALLIN, FORWARD, BACKWARD };

  bool calibrated = false;
  adc_oneshot_unit_handle_t adc1_handle;
  adc_cali_handle_t cali_handle = NULL;
  void setup_adc();
  float get_voltages();

  double gripper_speed_multiplier = 1.0;
  bool rollin = false;

  float prev_voltage = 0;

  double voltage_scale = 0.004307851239669421;

  FALLIN_STATE fallin_state = FALLIN_STATE::NOT_FALLIN;

  GRIP_STATE grip_state = GRIP_STATE::NOT_GRIPPIN;

  void set_controll_config(uint8_t mask, uint8_t value);
  MainContext();
  ServoManager Servos;
  MotorManager Wheels;

  Motor *mt_br;
  Motor *mt_bl;
  Motor *mt_fr;
  Motor *mt_fl;

  Servo *lifter1;
  Servo *lifter2;

  Servo *claw2;
  Servo *claw1;

  double _dir_x = 0;
  double _dir_y = 0;

  double turn = 0;

  PresistentConfig<uint8_t> controll_config;
};
