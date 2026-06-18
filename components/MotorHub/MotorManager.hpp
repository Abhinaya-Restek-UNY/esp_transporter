#pragma once
#include "PWMManager.hpp"
#include "soc/gpio_num.h"

class Motor {
  friend class MotorManager;

protected:
  Motor(mcpwm_cmpr_handle_t cmp, gpio_num_t A, gpio_num_t B, double scale);

public:
  void set_direction(int16_t direction);
  void update_scale(double new_scale, uint16_t max_period);

private:
  int16_t prev;
  gpio_num_t A;
  gpio_num_t B;
  mcpwm_cmpr_handle_t cmp;
  double scale = 1;
  int8_t polarity = 0;
  void set_speed(int16_t direction);
  void set_polarity(int16_t direction);
};

class MotorManager : public PWMManager {
public:
  MotorManager(int timer_group_id, unsigned int frequency, float max_voltage,
               float current_voltage, float target_voltage);

  Motor *create_motor(gpio_num_t GPIO_A, gpio_num_t GPIO_B, gpio_num_t PWM);

  void update_voltage(float voltage);

private:
  float max_voltage;
  float current_voltage;
  float target_voltage;
  ll<Motor> *last_motor = NULL;
};
