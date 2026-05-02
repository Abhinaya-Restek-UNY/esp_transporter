#pragma once
#include "Servo.hpp"

class Gripper {
private:
  Servo* servo;
  float current_angle = 0;
  float angle_step = 5.0;
  float max_angle;  // <-- tambah ini
  float min_angle;  // <-- tambah ini

public:
  Gripper(Servo* servo);
  void increase_angle();
  void decrease_angle();
  void lift();        // <-- tambah ini
  void drop();        // <-- tambah ini
  float get_angle();
};