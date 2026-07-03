#ifndef LCD_PROCESSOR_H
#define LCD_PROCESSOR_H

#include <stdint.h>
#include <stddef.h>

// extern uint8_t my_lcd; 

// Process LCD data
void process_lcd_data(uint8_t *data, size_t len);
void send_to_uart(int uart_num, const char *data);
void process_espnow_received_data(const char *received_data);

void set_lcd_type(void);


#endif // LCD_PROCESSOR_H
