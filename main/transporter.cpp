#include "Gamepad.hpp"
#include "MotorHub.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include <stdio.h>

extern "C" void app_main(void) {

  Gamepad gamepad;
  gamepad.start();

  MotorHub mt_hub(0, 21000);
  Motor *motor1 = mt_hub.create_motor(GPIO_NUM_19, GPIO_NUM_5, GPIO_NUM_18);
  Motor *motor2 = mt_hub.create_motor(GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26);
  Motor *motor3 = mt_hub.create_motor(GPIO_NUM_4, GPIO_NUM_2, GPIO_NUM_32);
  Motor *motor4 = mt_hub.create_motor(GPIO_NUM_15, GPIO_NUM_23, GPIO_NUM_14);

  mt_hub.start();
  int16_t direction = 0;
  int8_t increment = 75;
  int8_t add = increment;
  while (1) {
    motor1->set_direction(direction);
    motor2->set_direction(direction);
    if (direction > 32768 - increment - 1) {
      add = -increment;
    }

    if (direction < -(32768 - increment - 1)) {
      add = increment;
    }
    motor3->set_direction(direction);
    motor4->set_direction(direction);
    direction += add;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
