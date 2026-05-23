#pragma once
#include "Servo.hpp"

class Gripper {
private:
  Servo *claw;
  Servo *lifter;
  float current_angle = 0;
  float claw_step = 0.1;
  float lifted_angle = 0;
  float drop_angle = 0;

public:
  Gripper(Servo *claw, Servo *lifter, float claw_step, float lifted_angle,
          float drop_angle);
  void close();
  void open();
  void lift();
  void drop();
};
