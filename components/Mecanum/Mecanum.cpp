#include "Mecanum.hpp"
#include <cmath>

Mecanum::Mecanum(Motor *fr, Motor *fl, Motor *br, Motor *bl)
    : fr(fr), fl(fl), br(br), bl(bl) {

      };

void Mecanum::update(int16_t x, int16_t y, double turn) {

  this->hypot = std::hypot(x, y);

  double cosval = 0;
  double sinval = 0;
  if (this->hypot > 0.0) {
    cosval = (double)x / hypot;
    sinval = (double)y / hypot;
  }

  this->power = hypot / 32767.0;

  fl_val = ((sinval + cosval) * power + turn);
  fr_val = ((sinval - cosval) * power - turn);
  bl_val = ((sinval - cosval) * power + turn);
  br_val = ((sinval + cosval) * power - turn);

  double max_wheel = fabs(fl_val);
  if (fabs(fr_val) > max_wheel)
    max_wheel = fabs(fr_val);
  if (fabs(bl_val) > max_wheel)
    max_wheel = fabs(bl_val);
  if (fabs(br_val) > max_wheel)
    max_wheel = fabs(br_val);

  if (max_wheel > 1.0) {
    fl_val /= max_wheel;
    fr_val /= max_wheel;
    bl_val /= max_wheel;
    br_val /= max_wheel;
  }

  this->fl->set_direction(fl_val * 32767.0);
  this->fr->set_direction(fr_val * 32767.0);
  this->bl->set_direction(bl_val * 32767.0);
  this->br->set_direction(br_val * 32767.0);
};
