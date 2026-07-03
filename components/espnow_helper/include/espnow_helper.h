#ifndef ESPNOW_HELPER_H
#define ESPNOW_HELPER_H

#include "esp_err.h"
#include <stdint.h>

// Callback function type for ESP-NOW received data
typedef void (*espnow_data_callback_t)(const uint8_t *data, int len);

// Initialize ESP-NOW
esp_err_t espnow_init(espnow_data_callback_t callback);

// Send data via ESP-NOW
esp_err_t espnow_send(const uint8_t *data, size_t len);

#endif // ESPNOW_HELPER_H
