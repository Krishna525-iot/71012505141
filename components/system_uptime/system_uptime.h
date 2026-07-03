#pragma once
#include <stdint.h>
#include <stddef.h>



extern uint64_t uptime_seconds; // to be used in other files

// Initialize uptime system
void uptime_init(void);

// Start uptime task
void uptime_task(void *pvParameters);

// Reset uptime to zero
void reset_system_uptime(void);

// Check if UART packet is a reset command
void check_uart_packet(uint8_t *buf, size_t len);

// Check if string is a reset command
void check_uart_string(char *str);
