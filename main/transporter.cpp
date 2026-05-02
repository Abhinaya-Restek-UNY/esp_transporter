#include "Gamepad.hpp"
#include "IMU.hpp"
#include "Mecanum.hpp"
#include "MotorHub.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "nvs_flash.h"
#include "soc/gpio_num.h"
#include <cmath>
#include <stdio.h>

extern "C" void app_main(void) {

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  Gamepad gamepad;
  gamepad.start();

  IMU imu(GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_35);
  imu.start(6);

  MotorHub mt_hub(0, 20'000);
  mt_hub.start();

  Motor *motor1 = mt_hub.create_motor(GPIO_NUM_19, GPIO_NUM_5, GPIO_NUM_18);
  Motor *motor2 = mt_hub.create_motor(GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26);
  Motor *motor3 = mt_hub.create_motor(GPIO_NUM_4, GPIO_NUM_2, GPIO_NUM_32);
  Motor *motor4 = mt_hub.create_motor(GPIO_NUM_15, GPIO_NUM_23, GPIO_NUM_14);

  Mecanum::PIDConf pid_conf{.Kp = 1.5, .Ki = 0.5, .Kd = 0.1, .delta_t = 10};

  Mecanum mecanum(motor1, motor2, motor3, motor4, &imu, &pid_conf);

  Gamepad::joy_data_t orientation;
  Gamepad::joy_data_t direction;
  while (1) {

    if (gamepad.is_pressed(Gamepad::ButtonCode::PS_BUTTON)) {
      gamepad.zero_r_joy();
      gamepad.zero_l_joy();
    }

    gamepad.get_l_joy(&direction);
    gamepad.get_r_joy(&orientation);

    mecanum.set_angle(atan2(orientation.y, orientation.x));

    mecanum.set_direction(direction.x << 6, direction.y << 6);

    mecanum.update();

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
