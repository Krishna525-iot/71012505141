#include "uart_handler.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include "debug_control.h"

#define TAG "UART_HANDLER"
#define UART_RX_BUFFER_SIZE 500 // RX buffer size
#define UART_TX_BUFFER_SIZE 500   // TX buffer size (0 if not used)

// Debug control variable


static uart_data_callback_t uart_callback = NULL; // Callback function for received data
static QueueHandle_t uart_event_queue = NULL;     // Queue for UART events



/**
 * @brief Initialize UART
 *
 * @param uart_num UART number (e.g., UART_NUM_1, UART_NUM_2)
 * @param tx_pin TX pin number
 * @param rx_pin RX pin number
 * @param callback Function pointer for handling received UART data
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
esp_err_t uart_init(int uart_num, int tx_pin, int rx_pin, uart_data_callback_t callback)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Install UART driver
    esp_err_t ret = uart_driver_install(uart_num, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 10, &uart_event_queue, 0);
    if (ret != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    // Configure UART parameters
    ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    // Set the callback function for data processing
    uart_callback = callback;

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "UART%d initialized successfully", uart_num);
    }
    return ESP_OK;
}

/**
 * @brief UART task to handle events and data
 *
 * @param param UART number (e.g., UART_NUM_1, UART_NUM_2) passed as parameter
 */
void uart_task(void *param)
{
    int uart_num = (int)param;  // Cast parameter to UART number
    uart_event_t event;         // UART event structure
    uint8_t data[UART_RX_BUFFER_SIZE];

    while (1)
    {
        // Wait for UART events
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
                if (DEBUG_INFO)
                {
                    ESP_LOGI(TAG, "UART%d data received, length: %d", uart_num, event.size);
                }
                int len = uart_read_bytes(uart_num, data, event.size, pdMS_TO_TICKS(100));
                if (len > 0 && uart_callback)
                {
                    // Call the registered callback with received data
                    uart_callback(data, len);
                }
                break;

            case UART_FIFO_OVF:
                if (DEBUG_INFO)
                {
                    ESP_LOGW(TAG, "UART%d FIFO overflow detected", uart_num);
                }
                uart_flush_input(uart_num);
                xQueueReset(uart_event_queue);
                break;

            case UART_BUFFER_FULL:
                if (DEBUG_INFO)
                {
                    ESP_LOGW(TAG, "UART%d ring buffer full", uart_num);
                }
                uart_flush_input(uart_num);
                xQueueReset(uart_event_queue);
                break;

            case UART_PARITY_ERR:
                if (DEBUG_INFO)
                {
                    ESP_LOGW(TAG, "UART%d parity error detected", uart_num);
                }
                break;

            case UART_FRAME_ERR:
                if (DEBUG_INFO)
                {
                    ESP_LOGW(TAG, "UART%d frame error detected", uart_num);
                }
                break;

            default:
                if (DEBUG_INFO)
                {
                    ESP_LOGW(TAG, "Unhandled UART%d event type: %d", uart_num, event.type);
                }
                break;
            }
        }
    }
}

/**
 * @brief Start UART task
 *
 * @param uart_num UART number (e.g., UART_NUM_1, UART_NUM_2)
 * @param task_name Name of the UART task
 * @param stack_size Stack size for the task
 * @param priority Task priority
 * @return esp_err_t ESP_OK on success, otherwise an error code
 */
// Start a FreeRTOS task for UART
esp_err_t uart_start_task(int uart_num, const char *task_name, uint32_t stack_size, UBaseType_t priority)
{
    if (!uart_callback)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "UART callback is not set. Initialize UART before starting the task.");
        }
        return ESP_FAIL;
    }

    if (xTaskCreate(uart_task, task_name, stack_size, (void *)uart_num, priority, NULL) != pdPASS)
    {
        if (DEBUG_INFO)
        {
            ESP_LOGE(TAG, "Failed to create UART task");
        }
        return ESP_FAIL;
    }

    if (DEBUG_INFO)
    {
        ESP_LOGI(TAG, "UART task '%s' started successfully", task_name);
    }
    return ESP_OK;
}
