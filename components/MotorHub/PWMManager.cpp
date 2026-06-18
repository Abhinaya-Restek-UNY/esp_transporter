#include "PWMManager.hpp"
#include "Panic.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "soc/soc.h"
#include <cmath>
PWMManager::PWMManager(int timer_group_id, unsigned int frequency)
    : frequency(frequency) {

  uint32_t divider = (APB_CLK_FREQ / (65535 * frequency)) + 1;
  uint32_t res_hz = APB_CLK_FREQ / divider;
  this->period = res_hz / frequency;
  mcpwm_timer_config_t timer_config = {
      .group_id = timer_group_id,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = res_hz,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = this->period,
      .flags = {.update_period_on_empty = true},
  };

  this->timer_config = timer_config;

  ERR_CHECK(mcpwm_new_timer(&timer_config, &this->timer_handle));
}

void PWMManager::start() {
  ERR_CHECK(mcpwm_timer_enable(this->timer_handle));

  ERR_CHECK(
      mcpwm_timer_start_stop(this->timer_handle, MCPWM_TIMER_START_NO_STOP));
}

mcpwm_cmpr_handle_t PWMManager::create_pwm(gpio_num_t PWM) {

  mcpwm_oper_handle_t current_oper = nullptr;

  if (this->total_mcpwm % MAX_COMPARATORS_PER_OPERATOR == 0) {
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
    current_oper = last_operator->data;
  }

  mcpwm_cmpr_handle_t new_cmpr = nullptr;
  mcpwm_comparator_config_t cmpr_config = {.flags = {
                                               .update_cmp_on_tez = true,
                                           }};

  ERR_CHECK(mcpwm_new_comparator(current_oper, &cmpr_config, &new_cmpr));
  ERR_CHECK(
      mcpwm_comparator_set_compare_value(new_cmpr, 0)); // Start at 0 speed

  push_node(&last_comparator, new_cmpr);
  total_comparator++;

  mcpwm_gen_handle_t new_gen = nullptr;
  mcpwm_generator_config_t gen_config = {
      .gen_gpio_num = PWM,
  };

  ERR_CHECK(mcpwm_new_generator(current_oper, &gen_config, &new_gen));

  ERR_CHECK(mcpwm_generator_set_action_on_timer_event(
      new_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                            MCPWM_TIMER_EVENT_EMPTY,
                                            MCPWM_GEN_ACTION_HIGH)));

  ERR_CHECK(mcpwm_generator_set_action_on_compare_event(
      new_gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                              new_cmpr, MCPWM_GEN_ACTION_LOW)));

  push_node(&last_generator, new_gen);
  total_generator++;

  return new_cmpr;
}
