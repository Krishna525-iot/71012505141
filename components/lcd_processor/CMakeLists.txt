idf_component_register(SRCS "lcd_processor.c"
                       INCLUDE_DIRS "include"
                       REQUIRES lcd_commands espnow_helper uart_handler debug_control system_uptime)
