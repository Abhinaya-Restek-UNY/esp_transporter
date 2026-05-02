
#pragma once
#include "Motor.hpp"
#include "Servo.hpp"
#include "driver/mcpwm_timer.h"
#include "soc/gpio_num.h"

// --- Hardware Limits & Configuration ---
#define MAX_OPERATOR                                                           \
  3 ///< ESP32 supports a maximum of 3 MCPWM operators per group
#define MAX_COMPARATORS_PER_OPERATOR                                           \
  2 ///< Each operator has 2 comparators (can control 2 PWM signals)
#define MAX_GENERATORS_PER_OPERATOR                                            \
  2 ///< Each operator has 2 generators (ties comparator to an actual GPIO)
#define MAX_MOTOR                                                              \
  6 ///< Maximum total motors supported (3 operators * 2 comparators)

#define MAX_PWM                                                                \
  32768 ///< The abstract maximum PWM value passed from user-space (15-bit
        ///< logic)
#define INTERNAL_PWM_RESOLUTION                                                \
  4096 ///< 12-bit internal resolution for the actual MCPWM timer

/**
 * @brief Manages the MCPWM peripheral on the ESP32 to control multiple motors.
 * * The ESP-IDF v5 MCPWM API requires a specific hierarchy:
 * 1 Timer -> drives -> N Operators -> which contain -> Comparators & Generators
 * -> output to GPIO. This class acts as a factory, abstracting away the complex
 * setup and automatically sharing Operators between motors to maximize hardware
 * utilization.
 */
class MotorHub {
public:
  /**
   * @brief Configures the main MCPWM timer that will drive all child motors.
   * @param timer_group_id The MCPWM hardware group to use (usually 0 or 1).
   * @param frequency The desired PWM frequency in Hz.
   */
  MotorHub(int timer_group_id, unsigned int frequency);

  /**
   * @brief Factory method to create and allocate hardware for a new motor.
   * @param GPIO_A Motor direction pin A.
   * @param GPIO_B Motor direction pin B.
   * @param PWM Motor speed pin (tied to the MCPWM generator).
   * @return Motor* Pointer to the dynamically created Motor object, or nullptr
   * if hardware limits are reached.
   */
  Motor *create_motor(gpio_num_t GPIO_A, gpio_num_t GPIO_B, gpio_num_t PWM);

  /**
   * @brief Starts the underlying MCPWM timer, beginning PWM generation for all
   * created motors.
   */
  void start();

  Servo *create_servo(gpio_num_t PWM, uint16_t max_angle, float min_pulse_ms,
                      float max_pulse_ms);

private:
  //
  mcpwm_cmpr_handle_t create_pwm(gpio_num_t PWM);

  /**
   * @brief A generic Doubly Linked List node struct used internally for memory
   * tracking.
   */
  template <typename T> struct ll {
    ll *prev;
    T *data;
    ll *next;
  };

  // --- Linked List Tails ---
  // We keep track of the tails of the lists to easily append new hardware
  // handles.
  ll<mcpwm_oper_t> *last_operator = NULL;
  ll<mcpwm_cmpr_t> *last_comparator = NULL;
  ll<mcpwm_gen_t> *last_generator = NULL;
  ll<Motor> *last_motor = NULL;
  ll<Servo> *last_serv = NULL;

  // --- Hardware Counters ---
  int8_t total_operator = 0;
  int8_t total_comparator = 0;
  int8_t total_generator = 0;
  int8_t total_motor = 0;
  unsigned int frequency;

  // --- Global Timer Handles ---
  mcpwm_timer_handle_t timer_handle; ///< The master timer driving all operators
  mcpwm_timer_config_t
      timer_config; ///< Stored config so operators know which group to bind to

  /**
   * @brief Helper to append a new data node to the internal tracking linked
   * lists.
   * @param tail Pointer to the pointer of the list's tail.
   * @param new_data The data to store.
   */
  template <typename T> void push_node(ll<T> **tail, T *new_data);
};
