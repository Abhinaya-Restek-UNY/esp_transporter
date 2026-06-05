#pragma once

#include "nvs_flash.h"
#include <string>

template <typename T> class PresistentConfig {
public:
  // Constructor defined inline
  PresistentConfig(const std::string &key, T default_value)
      : key(key), handle(0) {
    size_t required_size = sizeof(T);
    this->value = default_value;
    auto ret = nvs_open("storage", NVS_READWRITE, &this->handle);
    if (ret == ESP_OK) {
      if (nvs_get_blob(handle, this->key.c_str(), &this->value,
                       &required_size) != ESP_OK ||
          required_size != sizeof(T)) {
        this->value = default_value;
      }
    } else {
      printf("Failed to open NVS: %s", esp_err_to_name(ret));
    }
  }

  T value;

  // Methods defined inline
  void save() {
    if (handle) {
      nvs_set_blob(handle, this->key.c_str(), &this->value, sizeof(T));
      nvs_commit(handle);
      printf("yooo i commit man\n");
    }
  }

  void remove() {
    if (handle) {
      nvs_erase_key(handle, this->key.c_str());
      nvs_commit(handle);
    }
  }

  // Destructor defined inline
  ~PresistentConfig() {
    if (handle) {
      nvs_close(handle);
    }
  }

private:
  std::string key;
  nvs_handle_t handle;
};
