#include "MotorManager.hpp"
#include "Panic.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cmpr.h"
#include "freertos/FreeRTOS.h"

Motor::Motor(mcpwm_cmpr_handle_t cmp, gpio_num_t A, gpio_num_t B, double scale)
    : A(A), B(B), cmp(cmp), scale(scale) {
  gpio_set_direction(A, GPIO_MODE_OUTPUT);
  gpio_set_direction(B, GPIO_MODE_OUTPUT);
  this->set_direction(0);
};

void Motor::set_direction(int16_t direction) {

  this->set_speed(direction);
  this->set_polarity(direction);
};

void Motor::set_speed(int16_t direction) {
  if (direction == 0) {
    ERR_CHECK(mcpwm_comparator_set_compare_value(this->cmp, 0));
    return;
  }

  ERR_CHECK(mcpwm_comparator_set_compare_value(
      this->cmp, (uint32_t)((float)abs(direction) * this->scale)));
}

void Motor::set_polarity(int16_t direction) {
  int8_t current_polarity = direction == 0 ? 0 : (direction > 0 ? 1 : -1);
  if (this->polarity == current_polarity) {
    return;
  }
  if ((this->polarity == 1 && current_polarity == -1) ||
      (this->polarity == -1 && current_polarity == 1)) {

    ERR_CHECK(mcpwm_comparator_set_compare_value(this->cmp, 0));

    gpio_set_level(this->A, 1);
    gpio_set_level(this->B, 1);

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  this->polarity = current_polarity;
  if (direction == 0) {
    gpio_set_level(this->A, 1); // Short brake
    gpio_set_level(this->B, 1); // Short brake
    return;
  } else if (direction > 0) {
    gpio_set_level(this->A, 1);
    gpio_set_level(this->B, 0);
  } else {
    gpio_set_level(this->A, 0);
    gpio_set_level(this->B, 1);
  }
}

void Motor::update_scale(double new_scale, uint16_t max_period) {
  this->scale = new_scale;
  if ((32768.0 * this->scale) > max_period) {
    this->scale = (double)max_period / 32768.0;
  }
}

MotorManager::MotorManager(int timer_group_id, unsigned int frequency,
                           float max_voltage, float current_voltage,
                           float target_voltage)
    : PWMManager(timer_group_id, frequency), max_voltage(max_voltage),
      current_voltage(current_voltage), target_voltage(target_voltage) {}

Motor *MotorManager::create_motor(gpio_num_t GPIO_A, gpio_num_t GPIO_B,
                                  gpio_num_t PWM) {
  if (total_mcpwm >= MAX_PWM) {
    return nullptr;
  }

  double voltage_ratio = (double)this->target_voltage / this->current_voltage;
  double base_scale = (double)this->period / 32768.0;
  double final_scale = base_scale * voltage_ratio;

  if ((32768.0 * final_scale) > this->period) {
    final_scale = (double)this->period / 32768.0;
  }

  Motor *new_motor = new Motor(create_pwm(PWM), GPIO_A, GPIO_B, final_scale);

  push_node(&last_motor, new_motor);
  total_mcpwm++;

  return new_motor;
}

void MotorManager::update_voltage(float voltage) {
  if (voltage <= 0.1f) {
    voltage = 0.1f;
  }

  this->current_voltage = voltage;

  double voltage_ratio = (double)this->target_voltage / this->current_voltage;
  double base_scale = (double)this->period / 32768.0;
  double new_scale = base_scale * voltage_ratio;

  ll<Motor> *curr = last_motor;
  while (curr != nullptr) {
    if (curr->data != nullptr) {
      curr->data->update_scale(new_scale, this->period);
    }
    curr = curr->prev;
  }
}
