#include "lcd_commands.h"
#include "lcd_processor.h"
#include "esp_log.h"
#include "espnow_helper.h"
#include <string.h>
#include "debug_control.h"
#include "uart_handler.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "system_uptime.h"

// Declare the uptime reset function from main.c
extern void reset_system_uptime(void);

#define TAG "LCD_PROCESSOR"
#define DATA_SIZE 9 // Expected size of received data

// Defaults
screen_type_t my_lcd = SCREEN_L;
uint8_t dst = 'L';
uint8_t isync = '0';

// ==============================

static const uint8_t laser_on_data[]  = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x5A, 0x00, 0x01};
static const uint8_t laser_off_data[] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x5A, 0x00, 0x00};

void process_laser_command(const char *received_data)
{
    if (received_data == NULL) {
        ESP_LOGW(TAG, "Received null Laser command pointer");
        return;
    }

    // Compare the first 5 bytes of the received command
    if (strncmp(received_data, "@PA1#", 5) == 0) {
        ESP_LOGI(TAG, "Laser ON command received via ESP-NOW");

        // Send Laser ON command to UART2
        ssize_t w = uart_write_bytes(UART_NUM_2, (const char *)laser_on_data, sizeof(laser_on_data));
        if (w < 0) ESP_LOGE(TAG, "UART2 write error for laser ON");
        ESP_LOGI(TAG, "Laser ON command sent to UART2");

    }
    else if (strncmp(received_data, "@PA0#", 5) == 0) {
        ESP_LOGI(TAG, "Laser OFF command received via ESP-NOW");

        // Send Laser OFF command to UART2
        ssize_t w = uart_write_bytes(UART_NUM_2, (const char *)laser_off_data, sizeof(laser_off_data));
        if (w < 0) ESP_LOGE(TAG, "UART2 write error for laser OFF");
        ESP_LOGI(TAG, "Laser OFF command sent to UART2");

    }
    else {
        ESP_LOGW(TAG, "Unknown Laser command received: %s", received_data);
    }
}

// ================================

void set_lcd_type(void) {
    const int GPIO_INPUT_PIN = 27;
    gpio_set_direction(GPIO_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_INPUT_PIN);

    int pin_state = gpio_get_level(GPIO_INPUT_PIN);
    if (pin_state == 1) {
        my_lcd = SCREEN_L;
        dst = 'L';
    } else {
        my_lcd = SCREEN_R;
        dst = 'R';
    }
}

void send_to_uart(int uart_num, const char *data) {
    if (data == NULL) return;
    // send full string (null-terminated) to provided UART (use uart_write_bytes wrapper)
    ssize_t w = uart_write_bytes((uart_port_t)uart_num, data, strnlen(data, 256));
    if (w < 0) {
        ESP_LOGE(TAG, "send_to_uart: uart_write_bytes returned error for uart %d", uart_num);
    }
}

void process_lcd_data(uint8_t *data, size_t len) {
    if (len == 0 || data == NULL) {
        ESP_LOGW(TAG, "UART read failed or returned no data.");
        return;
    }

    // Handle system reset command (exact 5 bytes string like "@RA0#"? original used 6: be tolerant)
    if (len >= 5 && memcmp(data, "@RA0#", 5) == 0) {
        uptime_seconds = 0;
        ESP_LOGI(TAG, "System uptime reset to 0 due to @RA0# packet");
        return;
    }

    if (len != DATA_SIZE) {
        ESP_LOGE(TAG, "Data length mismatch: expected %d, got %d", DATA_SIZE, (int)len);
        return;
    }

    ESP_LOGI(TAG, "Received Data:");
    ESP_LOG_BUFFER_HEX(TAG, data, len);

    for (int i = 0; i < lcd_command_map_size; i++) {
        // match by identifying bytes (same as original)
        if (data[5] == lcd_command_map[i].data[5] && data[8] == lcd_command_map[i].data[8]) {
            char formatted_data[32];
            char screen_char;

            // Correct mapping: dst must match the screen found (no reversal)
            switch (lcd_command_map[i].screen) {
                case SCREEN_L:
                    screen_char = 'L';
                    my_lcd = SCREEN_L;
                    dst = 'L';
                    break;
                case SCREEN_R:
                    screen_char = 'R';
                    my_lcd = SCREEN_R;
                    dst = 'R';
                    break;
                default:
                    ESP_LOGE(TAG, "Unknown screen type in lcd_command_map[%d]", i);
                    return;
            }

            // SYNC must not flip the target dome here. We simply append isync flag.
            if (isync == '1') {
                ESP_LOGI(TAG, "Sync ON (will include isync flag in broadcast)");
            } else {
                ESP_LOGI(TAG, "Sync OFF");
            }

            // Build the command to broadcast via ESP-NOW (e.g. "<CMD>W<L/R><isync>")
            snprintf(formatted_data, sizeof(formatted_data), "%sW%c%c",
                     lcd_command_map[i].command, screen_char, isync);
            formatted_data[sizeof(formatted_data) - 1] = '\0';

            // Send on local UART (for debugging / local processing)
            send_to_uart(UART_NUM_0, formatted_data);
            ESP_LOGI(TAG, "Processed Data (local): %s", formatted_data);

            // Broadcast via ESP-NOW
            esp_err_t err = espnow_send((uint8_t *)formatted_data,
                                        strnlen(formatted_data, sizeof(formatted_data)));
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "ESP-NOW broadcast failed: %s", esp_err_to_name(err));
                return;
            }
            ESP_LOGI(TAG, "Data broadcast successfully via ESP-NOW");

            return;
        }
    }

    ESP_LOGW(TAG, "Unrecognized LCD data");
    ESP_LOG_BUFFER_HEX(TAG, data, len);
}

// ---------------------- PROCESS RECEIVED ESPNOW COMMANDS ----------------------

// do not redefine screen_type globally if already defined elsewhere; keep a local one
void process_espnow_received_data(const char *received_data) {
    if (received_data == NULL) {
        ESP_LOGW(TAG, "process_espnow_received_data: null pointer");
        return;
    }

    size_t rlen = strlen(received_data);
    if (rlen < 5) {
        ESP_LOGW(TAG, "process_espnow_received_data: short packet");
        return;
    }

    // Handle laser/lamp commands immediately
    if (strncmp(received_data, "@PA1#", 5) == 0 ||
        strncmp(received_data, "@PA0#", 5) == 0) {
        process_laser_command(received_data);
        return;
    }

    // Expect format: "<CMD>W<X><Y>" where X is 'L'/'R' at index 6 and Y is isync at index 7
    // Guard against short strings
    char screen_ch = 'R';
    if (rlen > 6) screen_ch = received_data[6];
    isync = (rlen > 7 && received_data[7] == '1') ? '1' : '0';

    screen_type_t target_dome;

    // ---------- PRIORITY 1: explicit TL/TR anywhere in payload ----------
    if (strstr(received_data, "TL")) {
        target_dome = SCREEN_L;
    } else if (strstr(received_data, "TR")) {
        target_dome = SCREEN_R;
    } else {
        // ---------- PRIORITY 2: screen character decides ----------
        target_dome = (screen_ch == 'L') ? SCREEN_L : SCREEN_R;
    }

    // ESP_LOGI(TAG, "Received espnow: '%s' | screen_ch=%c isync=%c target_dome=%s",
    //          received_data, screen_ch, isync,
    //          (target_dome == SCREEN_L) ? "SCREEN_L" : "SCREEN_R");

    // ---------- FIND MATCHING COMMAND(S) ----------
    // If isync == '1', we will send the command to BOTH domes (if mappings exist)
    if (isync == '1') {
        // For sync ON, find matching entry for both screens and send each
        for (int s = 0; s < 2; s++) {
            screen_type_t send_screen = (s == 0) ? SCREEN_L : SCREEN_R;
            for (int i = 0; i < reverse_command_map_size; i++) {
                if (strncmp(received_data, reverse_command_map[i].command, 5) == 0 &&
                    reverse_command_map[i].screen == send_screen) {

                    const uint8_t *lcd_data = reverse_command_map[i].data;

                    // ALWAYS send to UART2 (actual dome device)
                    if (SEND_UART2) {
                        ssize_t w = uart_write_bytes(UART_NUM_2, (const char *)lcd_data, 8);
                        if (w < 0) ESP_LOGE(TAG, "UART2 write error when sending to screen %c", (send_screen == SCREEN_L) ? 'L' : 'R');
                    }

                    // Update local LCD if this ESP is that target dome
                    if (SEND_UART1 && (my_lcd == send_screen)) {
                        printf("%.*s\n", 5, received_data);
                    }
                    // we continue scanning to send other matching command(s)
                }
            }
        }
        return;
    } else {
        // isync == '0' -> only the target_dome should receive & update
        for (int i = 0; i < reverse_command_map_size; i++) {
            if (strncmp(received_data, reverse_command_map[i].command, 5) == 0 &&
                reverse_command_map[i].screen == target_dome) {

                const uint8_t *lcd_data = reverse_command_map[i].data;

                // ALWAYS send command to the target dome's UART (UART2)
                if (SEND_UART2) {
                    ssize_t w = uart_write_bytes(UART_NUM_2, (const char *)lcd_data, 8);
                    if (w < 0) ESP_LOGE(TAG, "UART2 write error when sending to target_dome");
                }

                // ONLY update local LCD if this board is the target dome
                if (SEND_UART1 && (my_lcd == target_dome)) {
                    printf("%.*s\n", 5, received_data);
                }

                return;
            }
        }
    }

    // If we reach here, no mapping found
    ESP_LOGW(TAG, "No reverse mapping found for espnow command: %s", received_data);
}

