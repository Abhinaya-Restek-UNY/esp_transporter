#pragma once
#include "Motor.hpp"
#include "driver/mcpwm_timer.h"
#include "soc/gpio_num.h"

#define MAX_OPERATOR 3
#define MAX_COMPARATORS_PER_OPERATOR 2
#define MAX_GENERATORS_PER_OPERATOR 2
#define MAX_MOTOR 6

#define MAX_PWM 32768
#define INTERNAL_PWM_RESOLUTION 4096

class MotorHub {
public:
  MotorHub(int timer_group_id, unsigned int frequency);
  Motor *create_motor(gpio_num_t GPIO_A, gpio_num_t GPIO_B, gpio_num_t PWM);

  void start();

private:
  friend Motor;
  template <typename T> struct ll {
    ll *prev;
    T *data;
    ll *next;
  };

  ll<mcpwm_oper_t> *last_operator = NULL;
  ll<mcpwm_cmpr_t> *last_comparator = NULL;
  ll<mcpwm_gen_t> *last_generator = NULL;
  ll<Motor> *last_motor = NULL;

  int8_t total_operator = 0;
  int8_t total_comparator = 0;
  int8_t total_generator = 0;
  int8_t total_motor = 0;

  mcpwm_timer_handle_t timer_handle;

  mcpwm_timer_config_t timer_config;

  template <typename T> void push_node(ll<T> **tail, T *new_data);
};
