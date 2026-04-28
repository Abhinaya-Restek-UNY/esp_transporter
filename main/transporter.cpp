#include "Gamepad.hpp"
#include "IMU.hpp"
#include "MotorHub.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "nvs_flash.h"
#include "soc/gpio_num.h"
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

  // MotorHub mt_hub(0, 21000);
  // Motor *motor1 = mt_hub.create_motor(GPIO_NUM_19, GPIO_NUM_5, GPIO_NUM_18);
  // Motor *motor2 = mt_hub.create_motor(GPIO_NUM_25, GPIO_NUM_27, GPIO_NUM_26);
  // Motor *motor3 = mt_hub.create_motor(GPIO_NUM_4, GPIO_NUM_2, GPIO_NUM_32);
  // Motor *motor4 = mt_hub.create_motor(GPIO_NUM_15, GPIO_NUM_23, GPIO_NUM_14);

  // mt_hub.start();
  // int16_t direction = 0;
  // int8_t increment = 75;
  // int8_t add = increment;
  // IMU imu(GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_35);
  // auto ret = imu.start(6);
  //
  // printf("ret %d\n", ret);
  // double yaw = 0;
  // double angular = 0;
  // uint64_t time = 0;
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
    // imu.get_yaw(&yaw, &angular, &time);
    //
    // printf("%f %f %llu\n", yaw * 180 / M_PI, angular, time);
  }
}
