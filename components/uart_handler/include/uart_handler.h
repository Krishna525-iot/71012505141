#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// Callback type for UART data
typedef void (*uart_data_callback_t)(uint8_t *data, size_t len);

// Initialize UART
esp_err_t uart_init(int uart_num, int tx_pin, int rx_pin, uart_data_callback_t callback);
esp_err_t uart_init_main(int uart_num, int tx_pin, int rx_pin);

// Start a FreeRTOS task for UART
esp_err_t uart_start_task(int uart_num, const char *task_name, uint32_t stack_size, UBaseType_t priority);

#endif // UART_HANDLER_H
