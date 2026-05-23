#pragma once

#include "Motor.hpp"

class Mecanum {
public:
  Mecanum(Motor *fr, Motor *fl, Motor *br, Motor *bl);

  void update(int16_t x, int16_t y, double turn);

private:
  double angle_target = 0.0;

  double power = 0.0;

  double hypot = 0.0;

  double fl_val = 0.0;
  double fr_val = 0.0;
  double bl_val = 0.0;
  double br_val = 0.0;

  Motor *fr;
  Motor *fl;
  Motor *br;
  Motor *bl;
};
