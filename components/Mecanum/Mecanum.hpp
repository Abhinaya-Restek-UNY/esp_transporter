#include "IMU.hpp"
#include "Motor.hpp"
#include "PID.hpp"
class Mecanum {
public:
  struct PIDConf {
    double Kp;
    double Ki;
    double Kd;
    uint64_t delta_t;
  };

  Mecanum(Motor *fr, Motor *fl, Motor *br, Motor *bl, IMU *imu,
          PIDConf *pid_conf);

  void set_angle(float yaw);

  void set_direction(int16_t x, int16_t y);

  void update();

private:
  double angle_target = 0.0;

  PID angle_pid;
  double power = 0.0;

  double hypot = 0.0;

  double turn = 0.0;

  double fl_val = 0.0;
  double fr_val = 0.0;
  double bl_val = 0.0;
  double br_val = 0.0;

  double yaw;
  double angular;
  uint64_t imu_timestamp;

  Motor *fr;
  Motor *fl;
  Motor *br;
  Motor *bl;

  IMU *imu;
};
