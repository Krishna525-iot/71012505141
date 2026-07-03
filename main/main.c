//every thin working okk but config mode is notworking

#include "uart_handler.h"
#include "espnow_helper.h"
#include "lcd_processor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "debug_control.h"
#include <string.h>
#include "lcd_commands.h"
#include "RCSwitch.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "system_uptime.h"

#define TAG "MAIN_APP"
#define RS433_TAG "RS433_TEST"
#define RECEIVER_GPIO 14

#define UART_NUM UART_NUM_2
#define UART2_TX_PIN 17
#define UART2_RX_PIN 16

#define UART1_NUM UART_NUM_1
#define UART1_TX_PIN 1
#define UART1_RX_PIN 3

#define GPIO_INPUT_PIN 27
#define CONFIG_LED_GPIO 25

#define CONFIG_BUTTON_PIN 26
#define CONFIG_MODE_TIMEOUT 6000000
#define MAX_VALUES 8

#define SCREEN_L 0
#define SCREEN_R 1

RCSWITCH_t myRCSwitch;

static volatile bool dome_selected = true;
static volatile int my_lcd = SCREEN_L;

static int intensity_dome1 = 1;
static int intensity_dome2 = 1;
static int color_dome1 = 1;
static int color_dome2 = 1;

static bool endo_state_dome1  = false;
static bool endo_state_dome2  = false;
static bool lamp_state_dome1  = false;
static bool lamp_state_dome2  = false;
static bool boost_state_dome1 = false;
static bool boost_state_dome2 = false;

/* ── sync mode state flag ───────────────────────────────────────────────── */
static volatile bool sync_mode_active = false;

bool is_dome1_command_allowed(void);
bool is_dome2_command_allowed(void);
void send_endo_command(bool state, bool dome1);
void send_lamp_command(bool state, bool dome1);
void send_boost_command(bool state, bool dome1);
void send_intensity_command(int intensity, bool dome1);
void send_color_command(int color, bool dome1);

/* ── forward declarations ───────────────────────────────────────────────── */
static void lcd_sync_reset_screens(uint8_t sync_val);
static void lcd_sync_apply(void);
static void lcd_sync_disable(void);

//--------------------------------------------------------------------------------------------------------
#define SUCCESS 0

#include "nvs_flash.h"
#include "nvs.h"

#define UART_RX_BUF_SIZE 64

typedef struct {
    unsigned long dec_intensity;
    unsigned long inc_intensity;
    unsigned long dec_color;
    unsigned long inc_color;
    unsigned long endo_toggle;
    unsigned long boost_toggle;
    unsigned long lamp_toggle;
    unsigned long dome_toggle;
} rf_mappings_t;

static rf_mappings_t current_mappings = {
    .dec_intensity = 16170421,
    .inc_intensity = 16170420,
    .dec_color     = 16170423,
    .inc_color     = 16170422,
    .endo_toggle   = 16170417,
    .boost_toggle  = 16170419,
    .lamp_toggle   = 16170418,
    .dome_toggle   = 16170424
};

static rf_mappings_t new_mappings;
static bool config_mode = false;
static TickType_t config_start_tick = 0;
static int mapping_to_update = 0;
static const char* mapping_names[MAX_VALUES] = {
    "Decrease Intensity",
    "Increase Intensity",
    "Decrease Color",
    "Increase Color",
    "Endo Toggle",
    "Boost Toggle",
    "Lamp Toggle",
    "Dome Toggle"
};

static esp_timer_handle_t led_timer = NULL;
static bool led_blink_state = false;

/* ─────────────────────────────────────────────────────────────────────────────
 *  lcd_sync_reset_screens()
 *
 *  Internal shared helper called by BOTH lcd_sync_apply() and lcd_sync_disable().
 *
 *  Sends the full baseline reset sequence to SCREEN_L then SCREEN_R, and at
 *  the very end writes the sync indicator register with `sync_val`:
 *      0x01  →  indicator ON  (sync mode entered)
 *      0x00  →  indicator OFF (sync mode exited)
 *
 *  Why a shared helper?
 *  The original lcd_sync_disable() had its macros #undef'd halfway through, so
 *  all the baseline packets after the #undef were never actually sent.  Sharing
 *  one body means the macros are defined once and undef'd once at the end,
 *  guaranteeing both paths execute the full reset identically.
 *
 *  Baseline applied to both screens on every call (ON and OFF):
 *    Intensity     → Level 1
 *    Color         → NW (@C05#)
 *    Lamp          → ON
 *    Endo          → OFF
 *    Depth/Boost   → OFF
 *    Field size    → OFF
 *    Overhead      → OFF
 *    Camera        → OFF
 *    Laser         → OFF
 *    Green mode    → ON, Green intensity → Level 1
 *    Red   mode    → ON, Red   intensity → Level 1
 *    Sync indicator→ sync_val  (0x01 = ON, 0x00 = OFF)
 * ───────────────────────────────────────────────────────────────────────────── */
static void lcd_sync_reset_screens(uint8_t sync_val)
{
    /* ── helpers: defined here, undef'd at the bottom of this function ─── */
    #define SEND8(a,b,c,d,e,f,g,h) \
        do { \
            uint8_t _p[8] = {(a),(b),(c),(d),(e),(f),(g),(h)}; \
            uart_write_bytes(UART_NUM, (const char*)_p, 8); \
            vTaskDelay(pdMS_TO_TICKS(5)); \
        } while(0)

    #define SEND_STR(s) \
        do { \
            uart_write_bytes(UART_NUM, (s), strlen(s)); \
            vTaskDelay(pdMS_TO_TICKS(5)); \
        } while(0)

    /* ════════════════════════════════════════════════════════════
       SCREEN L   (registers 0x00 – 0x3C)
       ════════════════════════════════════════════════════════════ */

    /* Intensity → Level 1 */
    SEND_STR("@I01#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x1E, 0x00,0x00);

    /* Color → NW */
    SEND_STR("@C05#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x02, 0x00,0x01);

    /* Lamp → ON */
    SEND_STR("@L_1#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x0A, 0x00,0x01);

    /* Endo → OFF */
    SEND_STR("@E_0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x07, 0x00,0x00);

    /* Depth / Boost → OFF */
    SEND_STR("@D_0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x1A, 0x00,0x00);

    /* Field size enable → OFF */
    SEND_STR("@F_0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x08, 0x00,0x00);

    /* Overhead sensor → OFF */
    SEND_STR("@O_0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x3C, 0x00,0x00);

    /* Camera → OFF */
    SEND_STR("@M_0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x09, 0x00,0x00);

    /* Laser → OFF */
    SEND_STR("@PA0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x5A, 0x00,0x00);

    /* Green → ON at intensity 1 */
    SEND_STR("@G_1#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x1C, 0x00,0x01);
    SEND_STR("@G01#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x03, 0x00,0x00);

    /* Red → ON at intensity 1 */
    SEND_STR("@R_1#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x06, 0x00,0x01);
    SEND_STR("@R01#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x05, 0x00,0x00);

    /* Sync indicator SCREEN_L (reg 0x6A) – ON or OFF per sync_val */
    if (sync_val) SEND_STR("@N_1#"); else SEND_STR("@N_0#");
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x6A, 0x00,sync_val);

    /* ════════════════════════════════════════════════════════════
       SCREEN R   (registers 0x41 – 0x6B)
       ════════════════════════════════════════════════════════════ */

    /* Intensity → Level 1 */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x41, 0x00,0x00);

    /* Color → NW */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x42, 0x00,0x01);

    /* Lamp → ON */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x4A, 0x00,0x01);

    /* Endo → OFF */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x47, 0x00,0x00);

    /* Depth / Boost → OFF */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x50, 0x00,0x00);

    /* Field size enable → OFF */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x48, 0x00,0x00);

    /* Overhead sensor → OFF */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x4C, 0x00,0x00);

    /* Camera → OFF */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x49, 0x00,0x00);

    /* Laser → OFF */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x5B, 0x00,0x00);

    /* Green → ON at intensity 1 */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x44, 0x00,0x01);
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x43, 0x00,0x00);

    /* Red → ON at intensity 1 */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x46, 0x00,0x01);
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x45, 0x00,0x00);

    /* Sync indicator SCREEN_R (reg 0x6B) – ON or OFF per sync_val */
    SEND8(0x5A,0xA5,0x05,0x82, 0x00,0x6B, 0x00,sync_val);

    /* ── in-memory tracking variables – always mirrors what we sent ─────── */
    intensity_dome1   = 1;      intensity_dome2   = 1;
    color_dome1       = 2;      color_dome2       = 2;   /* 2 = NW */
    endo_state_dome1  = false;  endo_state_dome2  = false;
    lamp_state_dome1  = true;   lamp_state_dome2  = true;
    boost_state_dome1 = false;  boost_state_dome2 = false;

    /* macros undef'd here – after all sends are complete */
    #undef SEND8
    #undef SEND_STR
}


/* ─────────────────────────────────────────────────────────────────────────────
 *  lcd_sync_apply()
 *  Called on SYNC ON (@N_1#).
 *  Resets both screens to baseline + turns sync indicator ON.
 * ───────────────────────────────────────────────────────────────────────────── */
static void lcd_sync_apply(void)
{
    ESP_LOGI(TAG, "SYNC ON → resetting both screens, indicator ON");
    sync_mode_active = true;
    lcd_sync_reset_screens(0x01);
    ESP_LOGI(TAG, "Sync ON complete");
}


/* ─────────────────────────────────────────────────────────────────────────────
 *  lcd_sync_disable()
 *  Called on SYNC OFF (@N_0#).
 *  Resets both screens to the same baseline + turns sync indicator OFF.
 * ───────────────────────────────────────────────────────────────────────────── */
static void lcd_sync_disable(void)
{
    ESP_LOGI(TAG, "SYNC OFF → resetting both screens, indicator OFF");
    sync_mode_active = false;
    lcd_sync_reset_screens(0x00);
    ESP_LOGI(TAG, "Sync OFF complete");
}


void led_on() {
    if (led_timer) esp_timer_stop(led_timer);
    gpio_set_level(CONFIG_LED_GPIO, 1);
}

void led_off() {
    if (led_timer) esp_timer_stop(led_timer);
    gpio_set_level(CONFIG_LED_GPIO, 0);
}

void led_blink_pattern(uint8_t count, uint32_t on_ms, uint32_t off_ms) {
    for(uint8_t i = 0; i < count; i++) {
        gpio_set_level(CONFIG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(CONFIG_LED_GPIO, 0);
        if(i < count-1) vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
    if(config_mode) {
        gpio_set_level(CONFIG_LED_GPIO, 1);
    }
}

void enter_config_mode() {
    config_mode = true;
    config_start_tick = xTaskGetTickCount();
    gpio_set_level(CONFIG_LED_GPIO, 1);
    ESP_LOGI(TAG, "=== CONFIG MODE STARTED ===");
    mapping_to_update = 0;
}

static void blink_led_fast(uint8_t times) {
    for (uint8_t i = 0; i < times; i++) {
        gpio_set_level(CONFIG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(CONFIG_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void blink_led_success(void) {
    for (uint8_t i = 0; i < 5; i++) {
        gpio_set_level(CONFIG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(CONFIG_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

#define RF_MAPPINGS_VERSION 1

void initialize_default_mappings() {
    current_mappings = (rf_mappings_t){
        .endo_toggle   = 16170417,
        .lamp_toggle   = 16170418,
        .boost_toggle  = 16170419,
        .inc_intensity = 16170420,
        .dec_intensity = 16170421,
        .inc_color     = 16170422,
        .dec_color     = 16170423,
        .dome_toggle   = 16170424
    };
    ESP_LOGI(RS433_TAG, "Initialized default mappings");
}

void save_new_mappings() {
    memcpy(&current_mappings, &new_mappings, sizeof(rf_mappings_t));
    ESP_LOGI(RS433_TAG, "Saving mappings to NVS...");

    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        err = nvs_set_u32(my_handle, "version", RF_MAPPINGS_VERSION);
        if (err != ESP_OK) {
            ESP_LOGE(RS433_TAG, "Failed to save version to NVS: %s", esp_err_to_name(err));
        }
        err = nvs_set_blob(my_handle, "rf_mappings", &current_mappings, sizeof(rf_mappings_t));
        if (err != ESP_OK) {
            ESP_LOGE(RS433_TAG, "Failed to save mappings to NVS: %s", esp_err_to_name(err));
        }
        nvs_commit(my_handle);
        nvs_close(my_handle);
    } else {
        ESP_LOGE(RS433_TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
    }
}

static void end_config_mode(bool saved) {
    if (saved) {
        ESP_LOGI(RS433_TAG, "Config saved. Exiting config mode");
        led_blink_pattern(4, 100, 60);
    }
    led_off();
    config_mode = false;
    save_new_mappings();
}

void load_mappings_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(rf_mappings_t);
        uint32_t version = 0;
        err = nvs_get_u32(my_handle, "version", &version);
        if (err == ESP_OK && version == RF_MAPPINGS_VERSION) {
            err = nvs_get_blob(my_handle, "rf_mappings", &current_mappings, &required_size);
            if (err == ESP_OK && required_size == sizeof(rf_mappings_t)) {
                ESP_LOGI(RS433_TAG, "Loaded mappings from NVS");
            } else {
                ESP_LOGE(RS433_TAG, "Failed to load mappings from NVS: %s", esp_err_to_name(err));
                initialize_default_mappings();
            }
        } else {
            ESP_LOGW(RS433_TAG, "No valid mappings found in NVS, using default values");
            initialize_default_mappings();
        }
        nvs_close(my_handle);
    } else {
        ESP_LOGE(RS433_TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
    }
}

static SemaphoreHandle_t mappings_mutex = NULL;

static void handle_config_mode(void) {
    if (!config_mode) return;

    if ((xTaskGetTickCount() - config_start_tick) >= pdMS_TO_TICKS(1500000)) {
        ESP_LOGI(RS433_TAG, "Config mode timeout");
        end_config_mode(false);
        return;
    }

    if (!available(&myRCSwitch)) return;

    unsigned long value = getReceivedValue(&myRCSwitch);
    resetAvailable(&myRCSwitch);
    if (value == 0) return;

    if (mapping_to_update < 0 || mapping_to_update >= MAX_VALUES) {
        ESP_LOGE(RS433_TAG, "mapping_to_update out of range (%d)", mapping_to_update);
        end_config_mode(false);
        return;
    }

    bool is_duplicate = false;
    if (mappings_mutex) xSemaphoreTake(mappings_mutex, pdMS_TO_TICKS(200));
    unsigned long *vals = (unsigned long*)&new_mappings;
    for (int i = 0; i < MAX_VALUES; i++) {
        if (vals[i] == value) { is_duplicate = true; break; }
    }
    if (mappings_mutex) xSemaphoreGive(mappings_mutex);

    if (is_duplicate) {
        ESP_LOGW(RS433_TAG, "Duplicate code %lu detected", value);
        led_blink_pattern(3,120,80);
        led_on();
        return;
    }

    if (mappings_mutex) xSemaphoreTake(mappings_mutex, pdMS_TO_TICKS(200));
    vals[mapping_to_update] = value;
    if (mappings_mutex) xSemaphoreGive(mappings_mutex);

    ESP_LOGI(RS433_TAG, "Learned %s -> %lu (index %d)",
             mapping_names[mapping_to_update], value, mapping_to_update);

    led_blink_pattern(2,100,80);
    led_off();
    vTaskDelay(pdMS_TO_TICKS(250));

    mapping_to_update++;
    if (mapping_to_update < MAX_VALUES) {
        ESP_LOGI(RS433_TAG, "Waiting for: %s", mapping_names[mapping_to_update]);
        led_on();
        return;
    }

    bool unique = true;
    unsigned long snapshot[MAX_VALUES];
    if (mappings_mutex) xSemaphoreTake(mappings_mutex, pdMS_TO_TICKS(200));
    for (int i = 0; i < MAX_VALUES; ++i) snapshot[i] = vals[i];
    if (mappings_mutex) xSemaphoreGive(mappings_mutex);

    for (int i = 0; i < MAX_VALUES && unique; ++i) {
        for (int j = i + 1; j < MAX_VALUES; ++j) {
            if (snapshot[i] == snapshot[j]) {
                unique = false;
                ESP_LOGE(RS433_TAG, "Duplicate learned mapping: %s and %s",
                         mapping_names[i], mapping_names[j]);
            }
        }
    }

    if (unique) {
        save_new_mappings();
        ESP_LOGI(RS433_TAG, "All mappings learned and saved");
        led_blink_pattern(4,100,60);
        end_config_mode(true);
    } else {
        ESP_LOGW(RS433_TAG, "Duplicates found after learning - not saved");
        led_blink_pattern(3,150,100);
        end_config_mode(false);
    }
}

bool is_dome1_command_allowed(void) {
    return (dome_selected == true) && (my_lcd == SCREEN_L);
}

bool is_dome2_command_allowed(void) {
    return (dome_selected == false) && (my_lcd == SCREEN_R);
}

static uint8_t reset_cmd_count = 0;

/* ─────────────────────────────────────────────────────────────────────────────
 *  uart_data_received()
 * ───────────────────────────────────────────────────────────────────────────── */
void uart_data_received(uint8_t *data, size_t len)
{
    if (!data || len == 0) return;

    /* ── uptime-reset detection (unchanged) ─────────────────────────────── */
    uint8_t reset_packet[9] = {0x5A,0xA5,0x06,0x83, 0x00,0x99, 0x01,0x00,0x00};
    bool is_reset_cmd = false;
    if (len >= 9 && memcmp(data, reset_packet, 9) == 0) {
        is_reset_cmd = true;
    } else if (len < UART_RX_BUF_SIZE) {
        char tmp[UART_RX_BUF_SIZE];
        memcpy(tmp, data, len);
        tmp[len] = '\0';
        if (strncmp(tmp, "@RA0#", 5) == 0) is_reset_cmd = true;
    }
    if (is_reset_cmd) {
        reset_cmd_count++;
        if (reset_cmd_count >= 4) { reset_system_uptime(); reset_cmd_count = 0; }
    } else {
        reset_cmd_count = 0;
    }

    /* ── SYNC detection ─────────────────────────────────────────────────────
     *  Shape A – 9-byte write (0x83), reg 0x6A or 0x6B
     *  Shape B – 8-byte reverse (0x82), reg 0x6A or 0x6B
     *  Shape C – plain text @N_1# / @N_0#
     * ──────────────────────────────────────────────────────────────────────── */
    bool sync_on_detected  = false;
    bool sync_off_detected = false;

    if (len >= 9) {
        if (data[0]==0x5A && data[1]==0xA5 && data[2]==0x06 &&
            data[3]==0x83 && data[4]==0x00 &&
            (data[5]==0x6A || data[5]==0x6B) &&
            data[6]==0x01 && data[7]==0x00) {
            if      (data[8] == 0x01) sync_on_detected  = true;
            else if (data[8] == 0x00) sync_off_detected = true;
        }
    }

    if (!sync_on_detected && !sync_off_detected && len >= 8) {
        if (data[0]==0x5A && data[1]==0xA5 && data[2]==0x05 &&
            data[3]==0x82 && data[4]==0x00 &&
            (data[5]==0x6A || data[5]==0x6B) &&
            data[6]==0x00) {
            if      (data[7] == 0x01) sync_on_detected  = true;
            else if (data[7] == 0x00) sync_off_detected = true;
        }
    }

    if (!sync_on_detected && !sync_off_detected && len >= 5) {
        char tmp[UART_RX_BUF_SIZE + 1];
        size_t copy_len = (len < UART_RX_BUF_SIZE) ? len : UART_RX_BUF_SIZE;
        memcpy(tmp, data, copy_len);
        tmp[copy_len] = '\0';
        if      (strstr(tmp, "@N_1#") != NULL) sync_on_detected  = true;
        else if (strstr(tmp, "@N_0#") != NULL) sync_off_detected = true;
    }

    if      (sync_on_detected)  lcd_sync_apply();
    else if (sync_off_detected) lcd_sync_disable();

    /* ── Forward to LCD processor (always) ───────────────────────────────── */
    process_lcd_data(data, len);
}

void uart1_data_received(uint8_t *data, size_t len) {
    if (DEBUG_INFO) {
        ESP_LOGI(TAG, "UART data received: %.*s", (int)len, (char *)data);
    }
    if (data && len > 0) {
        process_lcd_data(data, len);
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 *  espnow_data_received()
 * ───────────────────────────────────────────────────────────────────────────── */
void espnow_data_received(const uint8_t *data, int data_len) {
    char received_data[20] = {0};
    int copy_len = (data_len > (int)(sizeof(received_data) - 1))
                       ? (int)(sizeof(received_data) - 1)
                       : data_len;
    strncpy(received_data, (const char *)data, copy_len);

    if (DEBUG_INFO) {
        ESP_LOGI(TAG, "Received Data (as string): %s", received_data);
    }

    if (strstr(received_data, "@N_1#") != NULL) {
        ESP_LOGI(TAG, "ESP-NOW: SYNC ON from other dome");
        lcd_sync_apply();
    } else if (strstr(received_data, "@N_0#") != NULL) {
        ESP_LOGI(TAG, "ESP-NOW: SYNC OFF from other dome");
        lcd_sync_disable();
    }

    process_espnow_received_data(received_data);
}

void lcd_reset()
{
    char* data_arrays[] = {
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x02, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x0A, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x42, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x4A, 0x00, 0x01},
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x06, 0x00, 0x01}, // red l
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x1C, 0x00, 0x01}, // green l
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x44, 0x00, 0x01}, // green r
        (char[]){0x5A, 0xA5, 0x05, 0x82, 0x00, 0x46, 0x00, 0x01}  // red r
    };
    for (int i = 0; i < sizeof(data_arrays) / sizeof(data_arrays[0]); i++) {
        uart_write_bytes(UART_NUM_2, data_arrays[i], 8);
    }
}

void send_endo_command(bool state, bool dome1)
{
    char lcd_data[8] = {0x5A, 0xA5, 0x05, 0x82, 0x00, 0x00, 0x00, 0x00};
    char command[8];
    lcd_data[5] = dome1 ? 0x07 : 0x47;
    lcd_data[7] = state ? 0x01 : 0x00;
    if(state) strcpy(command, "@E_1#");
    else      strcpy(command, "@E_0#");
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char*)lcd_data, sizeof(lcd_data));
}

void send_lamp_command(bool state, bool dome1)
{
    char data[8];
    char command[8];
    if (dome1) {
        data[0]=0x5A; data[1]=0xA5; data[2]=0x05; data[3]=0x82;
        data[4]=0x00; data[5]=0x0A; data[6]=0x00; data[7]=state?0x01:0x00;
    } else {
        data[0]=0x5A; data[1]=0xA5; data[2]=0x05; data[3]=0x82;
        data[4]=0x00; data[5]=0x4A; data[6]=0x00; data[7]=state?0x01:0x00;
    }
    if(state) strcpy(command, "@L_1#");
    else      strcpy(command, "@L_0#");
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char*)data, sizeof(data));
}

void send_boost_command(bool state, bool dome1)
{
    char data[8];
    char command[8];
    if (dome1) {
        data[0]=0x5A; data[1]=0xA5; data[2]=0x05; data[3]=0x82;
        data[4]=0x00; data[5]=0x1A; data[6]=0x00; data[7]=state?0x01:0x00;
    } else {
        data[0]=0x5A; data[1]=0xA5; data[2]=0x05; data[3]=0x82;
        data[4]=0x00; data[5]=0x50; data[6]=0x00; data[7]=state?0x01:0x00;
    }
    if(state) strcpy(command, "@D_1#");
    else      strcpy(command, "@D_0#");
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char*)data, sizeof(data));
}

void send_intensity_command(int intensity, bool dome1)
{
    if (intensity < 1)  intensity = 1;
    if (intensity > 10) intensity = 10;

    char command[8];
    if (intensity == 10) strcpy(command, "@I0:#");
    else snprintf(command, sizeof(command), "@I%02d#", intensity);

    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);

    char data[8] = {0x5A,0xA5,0x05,0x82,0x00,0x00,0x00,(char)(intensity-1)};
    data[5] = dome1 ? 0x1E : 0x41;
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char*)data, sizeof(data));
}

void send_color_command(int color, bool dome1)
{
    char command[6];
    char data[8] = {0x5A,0xA5,0x05,0x82,0x00,0x00,0x00,0x00};

    if (color < 1) color = 1;
    if (color > 3) color = 3;

    switch (color) {
        case 1: strcpy(command,"@C-5#"); data[5]=dome1?0x02:0x42; data[7]=0x00; break;
        case 2: strcpy(command,"@C05#"); data[5]=dome1?0x02:0x42; data[7]=0x01; break;
        case 3: strcpy(command,"@C+5#"); data[5]=dome1?0x02:0x42; data[7]=0x02; break;
    }
    uart_write_bytes(UART_NUM_2, command, strlen(command));
    ESP_LOGI(RS433_TAG, "Sent command: %s", command);
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM_2, (const char*)data, sizeof(data));
}

static void gpio_dome_select_task(void *arg) {
    gpio_set_direction(GPIO_INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_INPUT_PIN);

    gpio_set_direction(CONFIG_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(CONFIG_BUTTON_PIN);

    bool last_gpio_state = gpio_get_level(GPIO_INPUT_PIN);
    dome_selected = (last_gpio_state == 1);
    my_lcd = dome_selected ? SCREEN_L : SCREEN_R;

    bool last_button = 1;
    TickType_t last_toggle_time = 0;

    for (;;) {
        int pin_state    = gpio_get_level(GPIO_INPUT_PIN);
        int button_state = gpio_get_level(CONFIG_BUTTON_PIN);

        if (pin_state != (int)last_gpio_state &&
            (xTaskGetTickCount() - last_toggle_time) > pdMS_TO_TICKS(200)) {
            last_gpio_state  = pin_state;
            last_toggle_time = xTaskGetTickCount();
            dome_selected    = (pin_state == 1);
            my_lcd           = dome_selected ? SCREEN_L : SCREEN_R;
            ESP_LOGI(TAG, "Dome selected via GPIO → Dome %d (LCD=%s)",
                     dome_selected ? 1 : 2, my_lcd == SCREEN_L ? "L" : "R");
        }

        if (button_state == 0 && last_button == 1 && !config_mode) {
            enter_config_mode();
        }
        last_button = button_state;

        if (config_mode) handle_config_mode();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void rs433_receiver_test_task(void *arg)
{
    ESP_LOGI(RS433_TAG, "Starting RS433 receiver test task");

    static int intensity1 = 1;
    static int intensity2 = 1;
    static int color1 = 2;
    static int color2 = 2;

    static unsigned long last_value = 0;
    static int64_t last_handled_time = 0;
    const int64_t DEBOUNCE_DELAY_US = 700000;

    initSwich(&myRCSwitch);
    enableReceive(&myRCSwitch, RECEIVER_GPIO);

    while (1)
    {
        if (available(&myRCSwitch))
        {
            unsigned long value = getReceivedValue(&myRCSwitch);
            int64_t now = esp_timer_get_time();

            if (value == 0) {
                resetAvailable(&myRCSwitch);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            if (value == last_value && (now - last_handled_time) < DEBOUNCE_DELAY_US) {
                resetAvailable(&myRCSwitch);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            last_value = value;
            last_handled_time = now;

            ESP_LOGI(RS433_TAG, "Received RF value: %lu", value);
            ESP_LOGI(RS433_TAG, "Current Dome: %s | GPIO Dome: %s",
                     dome_selected ? "Dome 1" : "Dome 2",
                     my_lcd == SCREEN_L ? "Dome 1" : "Dome 2");

            bool dome1_active = is_dome1_command_allowed();
            bool dome2_active = is_dome2_command_allowed();

            if (config_mode) {
                handle_config_mode();
                resetAvailable(&myRCSwitch);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            if (value == current_mappings.dome_toggle) {
                dome_selected = !dome_selected;
                ESP_LOGI(RS433_TAG, "Toggled Dome: Now using %s", dome_selected ? "Dome 1" : "Dome 2");
            }
            else if (value == current_mappings.endo_toggle && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1) { endo_state_dome1 = !endo_state_dome1; send_endo_command(endo_state_dome1, true); }
                else          { endo_state_dome2 = !endo_state_dome2; send_endo_command(endo_state_dome2, false); }
            }
            else if (value == current_mappings.lamp_toggle && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1) { lamp_state_dome1 = !lamp_state_dome1; send_lamp_command(lamp_state_dome1, true); }
                else          { lamp_state_dome2 = !lamp_state_dome2; send_lamp_command(lamp_state_dome2, false); }
            }
            else if (value == current_mappings.boost_toggle && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1) { boost_state_dome1 = !boost_state_dome1; send_boost_command(boost_state_dome1, true); }
                else          { boost_state_dome2 = !boost_state_dome2; send_boost_command(boost_state_dome2, false); }
            }
            else if (value == current_mappings.inc_intensity && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1 && intensity1 < 10) { intensity1++; send_intensity_command(intensity1, true); }
                else if (!is_dome1 && intensity2 < 10) { intensity2++; send_intensity_command(intensity2, false); }
            }
            else if (value == current_mappings.dec_intensity && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1 && intensity1 > 1) { intensity1--; send_intensity_command(intensity1, true); }
                else if (!is_dome1 && intensity2 > 1) { intensity2--; send_intensity_command(intensity2, false); }
            }
            else if (value == current_mappings.inc_color && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1 && color1 < 3) { color1++; send_color_command(color1, true); }
                else if (!is_dome1 && color2 < 3) { color2++; send_color_command(color2, false); }
            }
            else if (value == current_mappings.dec_color && (dome1_active || dome2_active)) {
                bool is_dome1 = dome1_active;
                if (is_dome1 && color1 > 1) { color1--; send_color_command(color1, true); }
                else if (!is_dome1 && color2 > 1) { color2--; send_color_command(color2, false); }
            }
            else {
                ESP_LOGW(RS433_TAG, "Unhandled RF code: %lu", value);
            }

            resetAvailable(&myRCSwitch);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));

    esp_reset_reason_t reason = esp_reset_reason();
    if (DEBUG_INFO) {
        ESP_LOGI(TAG, "Initializing application...");
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (DEBUG_INFO) {
            ESP_LOGW(TAG, "NVS flash issue detected, erasing...");
        }
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (DEBUG_INFO) { ESP_LOGI(TAG, "NVS initialized successfully"); }

    if (DEBUG_INFO) { ESP_LOGI(TAG, "Initializing UART..."); }
    ESP_ERROR_CHECK(uart_init(UART_NUM, UART2_TX_PIN, UART2_RX_PIN, uart_data_received));

    if (DEBUG_INFO) { ESP_LOGI(TAG, "Initializing ESP-NOW..."); }
    ESP_ERROR_CHECK(espnow_init(espnow_data_received));

    if (DEBUG_INFO) { ESP_LOGI(TAG, "Starting UART task..."); }
    ESP_ERROR_CHECK(uart_start_task(UART_NUM, "uart_task", 4096, 5));

    set_lcd_type();
    lcd_reset();

    gpio_reset_pin(CONFIG_LED_GPIO);
    gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
    led_off();

    enableReceive(&myRCSwitch, 4);
    load_mappings_from_nvs();

    gpio_reset_pin(CONFIG_LED_GPIO);
    gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_LED_GPIO, 0);
    xTaskCreate(gpio_dome_select_task, "gpio_dome_select_task", 4096, NULL, 5, NULL);
    xTaskCreate(rs433_receiver_test_task, "rs433_receiver_test_task", 4096, NULL, 5, NULL);

    BaseType_t res = xTaskCreate(
        rs433_receiver_test_task,
        "rs433_receiver_test_task",
        8192,
        NULL,
        5,
        NULL
    );
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RS433 receiver test task");
    }

    uptime_init();

    ESP_LOGI(TAG, "Starting tasks...");
    BaseType_t domeTask = xTaskCreate(gpio_dome_select_task, "gpio_dome_select_task", 4096, NULL, 5, NULL);
    BaseType_t rfTask   = xTaskCreate(rs433_receiver_test_task, "rs433_receiver_task", 4096, NULL, 5, NULL);

    if (domeTask == pdPASS && rfTask == pdPASS) {
        ESP_LOGI(TAG, "All tasks created successfully");
    } else {
        ESP_LOGE(TAG, "Failed to create one or more tasks!");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}