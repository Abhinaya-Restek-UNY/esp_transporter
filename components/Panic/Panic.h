#include "esp_err.h"
#include <stdio.h>
#include <stdlib.h>

// Define our custom error checker
#define ERR_CHECK(x)                                                           \
  do {                                                                         \
    esp_err_t err_rc_ = (x);                                                   \
    if (err_rc_ != ESP_OK) {                                                   \
      printf("ERROR: %s\n", esp_err_to_name(err_rc_));                         \
    }                                                                          \
  } while (0)
