#include "Motor.hpp"
// #define MECANUM_WITH_IMU

#ifdef MECANUM_WITH_IMU
#include "IMU.hpp"
#include "PID.hpp"
#endif
class Mecanum {
public:
  struct PIDConf {
    double Kp;
    double Ki;
    double Kd;
    uint64_t delta_t;
  };

  Mecanum(Motor *fr, Motor *fl, Motor *br, Motor *bl
#ifdef MECANUM_WITH_IMU
          ,
          IMU *imu, PIDConf *pid_conf
#endif
  );

  void set_direction(int16_t x, int16_t y);

  void update();

#ifdef MECANUM_WITH_IMU
  void set_angle(float yaw);
#else
  void set_turn(double amount);
#endif

private:
  double angle_target = 0.0;

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

#ifdef MECANUM_WITH_IMU
  IMU *imu;
  PID angle_pid;
#endif
};
