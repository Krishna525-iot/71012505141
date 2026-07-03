#include "system_uptime.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define LCD_HOURS_ADDR_H     0x00
#define LCD_HOURS_ADDR_L     0x77
#define LCD_MINUTES_ADDR_H   0x00
#define LCD_MINUTES_ADDR_L   0x78

// #define UART_RX_BUF_SIZE 64
#define TAG "UPTIME"

uint64_t uptime_seconds = 80;
static uint64_t last_nvs_save = 0;

//-------------------- LCD HELPERS --------------------
static void lcd_send_time_byte(uint8_t addr_h, uint8_t addr_l, uint8_t value) {
    uint8_t packet[8] = {0x5A, 0xA5, 0x05, 0x82, addr_h, addr_l, 0x00, value};
    uart_write_bytes(UART_NUM_2, (const char*)packet, 8);
}

static void lcd_send_time_word(uint8_t addr_h, uint8_t addr_l, uint16_t value) {
    uint8_t packet[8] = {0x5A, 0xA5, 0x06, 0x82, addr_h, addr_l, (value >> 8) & 0xFF, value & 0xFF};
    uart_write_bytes(UART_NUM_2, (const char*)packet, 8);
}

static void lcd_display_time(uint64_t seconds, bool force_hours) {
    static uint16_t last_hours = 65535;
    static uint8_t last_minutes = 255;

    uint16_t hours = seconds / 3600;
    uint8_t minutes = (seconds / 60) % 60;
    // ESP_LOGI(TAG, "Uptime: %02u hours, %02u minutes", hours, minutes);
    // Update hours only if forced (on boot) or hours changed
    if (force_hours || hours != last_hours) {
        lcd_send_time_word(LCD_HOURS_ADDR_H, LCD_HOURS_ADDR_L, hours);
        lcd_send_time_byte(LCD_MINUTES_ADDR_H, LCD_MINUTES_ADDR_L, minutes);
        lcd_send_time_byte(LCD_MINUTES_ADDR_H, LCD_MINUTES_ADDR_L, minutes);
        last_hours = hours;
    }

    // Always update minutes if changed
    if (minutes != last_minutes) {
        lcd_send_time_byte(LCD_MINUTES_ADDR_H, LCD_MINUTES_ADDR_L, minutes);
        last_minutes = minutes;
    }
}

//-------------------- NVS --------------------
static void save_uptime_to_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u64(handle, "uptime", uptime_seconds);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

static void load_uptime_from_nvs(void) {
    nvs_handle_t handle;
    uint64_t stored_uptime = 0;
    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        if (nvs_get_u64(handle, "uptime", &stored_uptime) == ESP_OK) {
            uptime_seconds = stored_uptime;
        } else {
            uptime_seconds = 0;
        }
        nvs_close(handle);
    } else {
        uptime_seconds = 0;
    }
}

//-------------------- RESET --------------------
void reset_system_uptime(void) {
    uptime_seconds = 0;
    last_nvs_save = 0;
    save_uptime_to_nvs();
    lcd_display_time(uptime_seconds, true);
}

static void handle_reset_command(void) {
    reset_system_uptime();
}

//-------------------- CHECK COMMAND --------------------

//-------------------- UPTIME TASK --------------------
void uptime_task(void *pvParameters) {
    load_uptime_from_nvs();
    last_nvs_save = uptime_seconds;

    // Send hours initially only on boot/reset
    lcd_display_time(uptime_seconds, true);

    TickType_t last_tick = xTaskGetTickCount();


    while (1) {
        TickType_t now_tick = xTaskGetTickCount();
        TickType_t elapsed_ticks = now_tick - last_tick;

        uint64_t elapsed_sec = (elapsed_ticks * portTICK_PERIOD_MS) / 1000;
        if (elapsed_sec > 0) {
            uptime_seconds += elapsed_sec;
            last_tick += (elapsed_sec * 1000) / portTICK_PERIOD_MS;

            // Update LCD time (hours updated only if changed)
            lcd_display_time(uptime_seconds, false);

            // Save uptime to NVS every 3 seconds
            if (uptime_seconds - last_nvs_save >= 3) {
                save_uptime_to_nvs();
                last_nvs_save = uptime_seconds;
            }
        }


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void uptime_init(void) {
    xTaskCreate(uptime_task, "uptime_task", 4096, NULL, 5, NULL);
}