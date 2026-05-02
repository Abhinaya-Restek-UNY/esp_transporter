#pragma once
#include "Servo.hpp"

class Gripper {
private:
  Servo* servo;
  float current_angle = 0;
  float angle_step = 5.0; // Increment/decrement per keypress

public:
  Gripper(Servo* servo);
  void increase_angle();
  void decrease_angle();
  float get_angle();
};