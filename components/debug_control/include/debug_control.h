#ifndef DEBUG_CONTROL_H
#define DEBUG_CONTROL_H

// Disable all debug logs
#define DEBUG_LCD   0
#define DEBUG_WIFI  0
#define DEBUG_INFO  0

// UART output control
// If you want to stop sending logs to LCD/ESP32 controllers, set these to 0
#define SEND_UART1  1   // set 0 if you want to stop UART1 output
#define SEND_UART2  1   // set 0 if you want to stop UART2 output

#endif // DEBUG_CONTROL_H
