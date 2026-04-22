#include "MotorHub.hpp"
#include "Motor.hpp"
#include "Panic.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
template <typename T> void MotorHub::push_node(ll<T> **tail, T *new_data) {
  ll<T> *newNode = new ll<T>{*tail, new_data, nullptr};
  if (*tail != nullptr) {
    (*tail)->next = newNode;
  }
  *tail = newNode; // Update the tail to be the new node
}

MotorHub::MotorHub(int timer_group_id, unsigned int frequency) {

  mcpwm_timer_config_t timer_config = {
      .group_id = 0,                          // MCPWM group 0
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, // Default is usually 80MHz
      .resolution_hz = frequency * 2048, // 80MHz resolution (1 tick = 12.5ns)
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = 2048, // 11-bit resolution
  };
  this->timer_config = timer_config;

  ERR_CHECK(mcpwm_new_timer(&timer_config, &this->timer_handle));
}

void MotorHub::start() {
  ERR_CHECK(mcpwm_timer_enable(this->timer_handle));

  ERR_CHECK(
      mcpwm_timer_start_stop(this->timer_handle, MCPWM_TIMER_START_NO_STOP));
}

Motor *MotorHub::create_motor(gpio_num_t GPIO_A, gpio_num_t GPIO_B,
                              gpio_num_t PWM) {
  if (total_motor >= MAX_MOTOR) {
    return nullptr;
  }

  mcpwm_oper_handle_t current_oper = nullptr;

  // 1. Do we need a new Operator?
  // Since 1 operator handles 2 motors, we need a new one on motor 0, 2, 4...
  if (total_motor % MAX_COMPARATORS_PER_OPERATOR == 0) {
    if (total_operator >= MAX_OPERATOR) {
      return nullptr;
    }

    mcpwm_operator_config_t oper_config = {
        .group_id = this->timer_config.group_id,
    };

    ERR_CHECK(mcpwm_new_operator(&oper_config, &current_oper));
    ERR_CHECK(mcpwm_operator_connect_timer(current_oper, this->timer_handle));

    push_node(&last_operator, current_oper);
    total_operator++;
  } else {
    // Reuse the last operator we created
    current_oper = last_operator->data;
  }

  // 2. Create the Comparator
  mcpwm_cmpr_handle_t new_cmpr = nullptr;
  mcpwm_comparator_config_t cmpr_config = {.flags = {
                                               .update_cmp_on_tez = true,
                                           }};

  ERR_CHECK(mcpwm_new_comparator(current_oper, &cmpr_config, &new_cmpr));
  ERR_CHECK(
      mcpwm_comparator_set_compare_value(new_cmpr, 0)); // Start at 0 speed

  push_node(&last_comparator, new_cmpr);
  total_comparator++;

  // 3. Create the Generator (Link to the PWM pin)
  mcpwm_gen_handle_t new_gen = nullptr;
  mcpwm_generator_config_t gen_config = {
      .gen_gpio_num = PWM,
  };

  ERR_CHECK(mcpwm_new_generator(current_oper, &gen_config, &new_gen));

  // Set Generator Actions (High on empty, Low on comparator match)
  ERR_CHECK(mcpwm_generator_set_action_on_timer_event(
      new_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            MCPWM_TIMER_EVENT_EMPTY,
                                            MCPWM_GEN_ACTION_HIGH)));

  ERR_CHECK(mcpwm_generator_set_action_on_compare_event(
      new_gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                              new_cmpr, MCPWM_GEN_ACTION_LOW)));

  push_node(&last_generator, new_gen);
  total_generator++;

  // 4. Create the Motor Object
  Motor *new_motor = new Motor(new_cmpr, GPIO_A, GPIO_B);

  push_node(&last_motor, new_motor);
  total_motor++;

  return new_motor;
}
