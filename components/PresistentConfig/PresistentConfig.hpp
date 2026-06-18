#pragma once

#include "nvs_flash.h"
#include <cstring>
#include <string>
#include <type_traits>

template <typename T> class PresistentConfig {
public:
  PresistentConfig(const std::string &key, const T &default_value)
      : key(key), handle(0) {
    size_t required_size = sizeof(T);

    if constexpr (std::is_array_v<T>) {
      std::memcpy(&this->value, &default_value, sizeof(T));
    } else {
      this->value = default_value;
    }

    auto ret = nvs_open("storage", NVS_READWRITE, &this->handle);
    if (ret == ESP_OK) {
      if (nvs_get_blob(handle, this->key.c_str(), &this->value,
                       &required_size) != ESP_OK ||
          required_size != sizeof(T)) {

        if constexpr (std::is_array_v<T>) {
          std::memcpy(&this->value, &default_value, sizeof(T));
        } else {
          this->value = default_value;
        }
      }
    } else {
      printf("Failed to open NVS: %s\n", esp_err_to_name(ret));
    }
  }

  T value;

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

  ~PresistentConfig() {
    if (handle) {
      nvs_close(handle);
    }
  }

private:
  std::string key;
  nvs_handle_t handle;
};
