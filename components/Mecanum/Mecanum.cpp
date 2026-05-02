#include "Mecanum.hpp"
#include <cmath>

Mecanum::Mecanum(Motor *fr, Motor *fl, Motor *br, Motor *bl, IMU *imu,
                 PIDConf *pid_conf)
    : angle_pid(pid_conf->Kp, pid_conf->Ki, pid_conf->Kp, pid_conf->delta_t,
                -1.0, 1.0, -0.5, 0.5),
      fr(fr), fl(fl), br(br), bl(bl), imu(imu) {

      };

void Mecanum::set_angle(float yaw) { this->angle_target = yaw; };

void Mecanum::set_direction(int16_t x, int16_t y) {
  this->hypot = sqrt((double)x * x + (double)y * y);
  if (this->hypot == 0.0) {
    this->power = 0.0;
    this->fl_val = 0.0;
    this->fr_val = 0.0;
    this->bl_val = 0.0;
    this->br_val = 0.0;
    return;
  }

  double cosval = (double)x / hypot;
  double sinval = (double)y / hypot;

  this->power = hypot / 32768.0;
  this->fl_val = (sinval + cosval) * power;
  this->fr_val = (sinval - cosval) * power;
  this->bl_val = (sinval - cosval) * power;
  this->br_val = (sinval + cosval) * power;
};

void Mecanum::update() {

  this->imu->get_yaw(&this->yaw, &this->angular, &this->imu_timestamp);
  printf("angular: %lf rad/s\n", this->angular);

  this->turn =
      this->angle_pid.update(this->angle_target, this->yaw, this->angular);

  double fl_combined = (this->fl_val + this->turn);
  double fr_combined = (this->fr_val - this->turn);
  double bl_combined = (this->bl_val + this->turn);
  double br_combined = (this->br_val - this->turn);

  double max_wheel = fabs(fl_combined);
  if (fabs(fr_combined) > max_wheel)
    max_wheel = fabs(fr_combined);
  if (fabs(bl_combined) > max_wheel)
    max_wheel = fabs(bl_combined);
  if (fabs(br_combined) > max_wheel)
    max_wheel = fabs(br_combined);

  if (max_wheel > 1.0) {
    fl_combined /= max_wheel;
    fr_combined /= max_wheel;
    bl_combined /= max_wheel;
    br_combined /= max_wheel;
  }

  this->fl->set_direction(fl_combined * 32767.0);
  this->fr->set_direction(fr_combined * 32767.0);
  this->bl->set_direction(bl_combined * 32767.0);
  this->br->set_direction(br_combined * 32767.0);
};
