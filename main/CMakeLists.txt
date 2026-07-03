idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash esp_wifi esp_event espnow_helper lcd_processor uart_handler debug_control RCSwitch system_uptime)