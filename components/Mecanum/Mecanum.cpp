#include "Mecanum.hpp"
#include <algorithm>
#include <cmath>

Mecanum::Mecanum(Motor *fr, Motor *fl, Motor *br, Motor *bl)
    : fr(fr), fl(fl), br(br), bl(bl) {

      };

void Mecanum::update(int16_t x, int16_t y, double turn) {
  double norm_x = x / 32767.0;
  double norm_y = y / 32767.0;

  fl_val = norm_y + norm_x + turn;
  fr_val = norm_y - norm_x - turn;
  bl_val = norm_y - norm_x + turn;
  br_val = norm_y + norm_x - turn;

  double max_wheel =
      std::max({1.0, fabs(fl_val), fabs(fr_val), fabs(bl_val), fabs(br_val)});

  double scale = 32767.0 / max_wheel;

  this->fl->set_direction(fl_val * scale);
  this->fr->set_direction(fr_val * scale);
  this->bl->set_direction(bl_val * scale);
  this->br->set_direction(br_val * scale);
}
