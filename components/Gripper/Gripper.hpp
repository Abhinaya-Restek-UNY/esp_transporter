
#pragma once
#include "PresistentConfig.hpp"
#include "ServoManager.hpp"
#include "esp_timer.h"

class Gripper {
private:
  Servo *claw;
  Servo *lifter;

public:
  Gripper(Servo *claw, Servo *lifter, float max_claw,
          const std::string &config_key);

  PresistentConfig<float> max_angle;
  void set_claw(bool gripping);
  void set_lifter(int16_t throttlebreak);
};
