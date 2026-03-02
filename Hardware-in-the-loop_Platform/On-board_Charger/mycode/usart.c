

#include "optimization.h"
#include "usart.h"
#include "stm32g4xx_hal.h"
#include "math.h"
#include "string.h"

UART_HandleTypeDef huart_comm;

__IO uint8_t uart_tx_buffer[UART_BUFFER_SIZE]; 
__IO uint8_t uart_rx_buffer[UART_BUFFER_SIZE]; 
__IO uint8_t rx_complete_flag = 0; 
__IO uint16_t rx_data_length = 0; 
__IO uint16_t rx_index = 0;              

uint16_t calculate_crc16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for(uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t j = 0; j < 8; j++) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return ((crc << 8) | (crc >> 8)); 
}

void uart_comm_init(void) {
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart_comm.Instance = USART2;
    huart_comm.Init.BaudRate = 115200;
    huart_comm.Init.WordLength = UART_WORDLENGTH_8B;
    huart_comm.Init.StopBits = UART_STOPBITS_1;
    huart_comm.Init.Parity = UART_PARITY_NONE;
    huart_comm.Init.Mode = UART_MODE_TX_RX;
    huart_comm.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_comm.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if(HAL_UART_Init(&huart_comm) != HAL_OK) {
        
        while(1);
    }

    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    __HAL_UART_ENABLE_IT(&huart_comm, UART_IT_RXNE);

    memset((void*)lower_variable, 0, sizeof(lower_variable));
    memset((void*)uart_tx_buffer, 0, sizeof(uart_tx_buffer));
    memset((void*)uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    rx_index = 0;
    rx_complete_flag = 0;
    communicate_sign = 0;
}

float calculate_distance(float *arr1, float *arr2, uint16_t size) {
    float sum = 0.0f;
    for(uint16_t i = 0; i < size; i++) {
        float diff = arr1[i] - arr2[i];
        sum += diff * diff;
    }
    return sqrtf(sum);
}

uint8_t check_communication_needed(float *current_power, uint16_t duration) {
    float distance = calculate_distance(lower_variable, current_power, duration);
    
    if(distance > COMM_THRESHOLD) {
        communicate_sign = 1;
        
        for(uint16_t t = 0; t < duration && t < MAX_HOUR; t++) {
            lower_variable[t] = current_power[t];
        }
        return 1; 
    } else {
        communicate_sign = 0;
        return 0; 
    }
}

void send_lower_variable(float *power_data, uint16_t length) {
    
    uint16_t data_bytes = length * 4; 
    uint16_t index = 0;
    uint16_t crc;

    memset((void*)uart_tx_buffer, 0, sizeof(uart_tx_buffer));

    uart_tx_buffer[index++] = FRAME_HEADER;      
    uart_tx_buffer[index++] = CMD_SEND_LOWER;    
    uart_tx_buffer[index++] = data_bytes;        

    for(uint16_t i = 0; i < length && i < MAX_HOUR; i++) {
        union {
            float f_val;
            uint8_t bytes[4];
        } converter;
        
        converter.f_val = power_data[i];

        uart_tx_buffer[index++] = converter.bytes[0];
        uart_tx_buffer[index++] = converter.bytes[1];
        uart_tx_buffer[index++] = converter.bytes[2];
        uart_tx_buffer[index++] = converter.bytes[3];
    }

    crc = calculate_crc16((uint8_t*)uart_tx_buffer, index);
    uart_tx_buffer[index++] = (uint8_t)(crc >> 8);   
    uart_tx_buffer[index++] = (uint8_t)(crc & 0xFF); 

    HAL_UART_Transmit(&huart_comm, (uint8_t*)uart_tx_buffer, index, HAL_MAX_DELAY);
}

void parse_received_frame(void) {
    
    if(rx_data_length < 5) return; 

    if(uart_rx_buffer[0] != FRAME_HEADER || uart_rx_buffer[1] != CMD_RECV_UPPER) {
        return;
    }
    
    uint16_t payload_length = uart_rx_buffer[2];
    uint16_t expected_length = payload_length + 5; 
    
    if(rx_data_length != expected_length) {
        return; 
    }

    uint16_t received_crc = ((uint16_t)uart_rx_buffer[rx_data_length-2] << 8) | uart_rx_buffer[rx_data_length-1];
    uint16_t calculated_crc = calculate_crc16((uint8_t*)uart_rx_buffer, rx_data_length-2);
    
    if(received_crc != calculated_crc) {
        return; 
    }

    uint16_t data_pairs = payload_length / 8; 
    uint16_t index = 3;
    
    for(uint16_t i = 0; i < data_pairs && i < MAX_HOUR; i++) {
        union {
            float f_val;
            uint8_t bytes[4];
        } converter;

        converter.bytes[0] = uart_rx_buffer[index++];
        converter.bytes[1] = uart_rx_buffer[index++];
        converter.bytes[2] = uart_rx_buffer[index++];
        converter.bytes[3] = uart_rx_buffer[index++];
        upper_variable[i] = converter.f_val;

        converter.bytes[0] = uart_rx_buffer[index++];
        converter.bytes[1] = uart_rx_buffer[index++];
        converter.bytes[2] = uart_rx_buffer[index++];
        converter.bytes[3] = uart_rx_buffer[index++];
        dual[i] = converter.f_val;
    }
}

void process_uart_data(void) {
    if(rx_complete_flag) {
        parse_received_frame();

        rx_complete_flag = 0;
        rx_index = 0;
        rx_data_length = 0;
        memset((void*)uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    }
}

void event_triggered_communicate(float *current_power, uint16_t duration) {
    
    if(check_communication_needed(current_power, duration)) {
        
        uint16_t send_length = (duration < period) ? duration : period;
        send_lower_variable(lower_variable, send_length);
    }
}

void USART2_IRQHandler(void) {
    
    if(__HAL_UART_GET_FLAG(&huart_comm, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart_comm, UART_FLAG_RXNE);
    
        uint8_t received_byte = (uint8_t)(huart_comm.Instance->RDR & 0xFF);
        if(rx_index == 0 && received_byte != FRAME_HEADER) {
            
            return;
        }
        
        if(rx_index < UART_BUFFER_SIZE) {
            uart_rx_buffer[rx_index++] = received_byte;

            if(rx_index >= 3) {
                uint16_t expected_total = uart_rx_buffer[2] + 5; 
                
                if(rx_index >= expected_total) {
                    rx_data_length = rx_index;
                    rx_complete_flag = 1;
                }
            }
        } else {
            
            rx_index = 0;
        }
    }
    
    if(__HAL_UART_GET_FLAG(&huart_comm, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_FLAG(&huart_comm, UART_FLAG_ORE);
    }
}

