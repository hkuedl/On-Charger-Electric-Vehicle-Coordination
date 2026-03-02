#ifndef __UART_COMM_H__ 
#define __UART_COMM_H__ 

#include "common.h" 

void uart_comm_init(void); 

void send_lower_variable(float *power_data, uint16_t length); 

void process_uart_data(void); 

void event_triggered_communicate(float *current_power, uint16_t duration); 

uint8_t check_communication_needed(float *current_power, uint16_t duration); 

#define UART_BUFFER_SIZE 200
#define COMM_THRESHOLD 0.2f
#define MAX_HOUR 96

#define FRAME_HEADER 0xAA
#define CMD_SEND_LOWER 0x01          
#define CMD_RECV_UPPER 0x02          

#endif
