#include "espnow_helper.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include <string.h>
#include "debug_control.h"

static const char *TAG = "ESPNOW_HELPER";

// Debug control variable
// #define DEBUG_ESPNOW 1  // Set to 1 to enable logs, 0 to disable

const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// ESP-NOW Receive Callback
static espnow_data_callback_t app_callback = NULL;

// ESP-NOW receive callback
// Wrapper for ESP-NOW receive callback
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int data_len)
{
    if (data == NULL || data_len <= 0)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGW(TAG, "Invalid ESP-NOW data received");
        }
        return;
    }

    // Log MAC address if required
    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "ESP-NOW data received, len: %d", data_len);
    }

    // Pass the data to the application callback
    if (app_callback)
    {
        app_callback(data, data_len);  // Pass 'data' (uint8_t*) and 'data_len' (int)
    }
}

// ESP-NOW Initialization
esp_err_t espnow_init(espnow_data_callback_t callback)
{
    esp_err_t err;

    // Register application callback
    app_callback = callback;

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Initializing ESP-NOW...");
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize Wi-Fi in STA mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // Register ESP-NOW receive callback
    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to register ESP-NOW receive callback: %s", esp_err_to_name(err));
        }
        return err;
    }

    // Set primary master key (PMK)
    uint8_t pmk[16] = "pmk1234567890123";
    ESP_ERROR_CHECK(esp_now_set_pmk(pmk));

    // Add broadcast peer
    esp_now_peer_info_t peer_info = {};
    memset(&peer_info, 0, sizeof(esp_now_peer_info_t));
    peer_info.channel = 0; // Use current Wi-Fi channel
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);

    err = esp_now_add_peer(&peer_info);
    if (err != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(err));
        }
        return err;
    }

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    }

    return ESP_OK;
}

// ESP-NOW Data Send
esp_err_t espnow_send(const uint8_t *data, size_t len)
{
    if (!data || len <= 0)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Invalid data to send");
        }
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = esp_now_send(broadcast_mac, data, len);
    if (err != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to send data via ESP-NOW: %s", esp_err_to_name(err));
        }
        return err;
    }

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "Data sent via ESP-NOW successfully");
    }

    return ESP_OK;
}


