
#include "STP_Decode.h"
#include "stm32f4xx_it.h"
#include <string.h>

/**************STP_23L********************/
uint8_t Rxbuffer_1[195];
uint8_t Rxbuffer_2[195];
uint8_t Rxbuffer_3[195];
uint8_t Rxbuffer_4[195];
uint8_t Rxbuffer_6[195];

LidarPointTypedef Lidar1;
LidarPointTypedef Lidar2;
LidarPointTypedef Lidar3;
LidarPointTypedef Lidar4;
LidarPointTypedef Lidar6;

uint16_t UartFlag[6];

uint8_t usart1_rx[1];
uint8_t usart2_rx[1];
uint8_t usart3_rx[1];
uint8_t usart4_rx[1];
uint8_t usart6_rx[1];
/*********************STP_23L***********************/
/*********************Ras_pi************************/
uint8_t receive_buffer[24];
float weight_placement[5] = {0};

volatile uint32_t usart1_rx_bytes = 0;
volatile uint32_t usart1_frame_cnt = 0;
volatile uint32_t usart3_frame_cnt = 0;

volatile uint8_t pi_digit_ready = 0;   // 收到数字串
volatile uint8_t pi_bean_ready  = 0;   // 收到豆子码
char pi_digit_str[16] = {0};           // 数字串 如 "D12345"
char pi_bean_code = 0;                 // 豆子码 '1'/'2'/'3'/'0'
/*********************Ras_pi************************/

float weight_placement_tmp[5] = {0};
uint16_t tar_count;
//uint16_t success_cnt;
/*********************Ras_pi************************/

uint16_t inner_ring_flag;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    usart1_rx_bytes++;
    static uint16_t u1state = 0;
    static uint16_t crc1    = 0;
    uint8_t tmp1 = usart1_rx[0];

    if (u1state < 4) {
        if (tmp1 == 0xAA) {
            Rxbuffer_1[u1state] = tmp1;
            u1state++;
        } else {
            u1state = 0;
        }
    } else if (u1state < 194) {
        Rxbuffer_1[u1state] = tmp1;
        u1state++;
        crc1 += tmp1;
    } else if (u1state == 194) {
        Rxbuffer_1[u1state] = tmp1;
        if (tmp1 == crc1 % 256) {
            UartFlag[0] = 1;
            usart1_frame_cnt++;
        }
        u1state = 0;
        crc1    = 0;
    }
    HAL_UART_Receive_IT(&huart1, usart1_rx, 1);
    }

    if (huart->Instance == USART3) {
        static uint16_t u3state = 0;
        static uint16_t crc3    = 0;
        uint8_t tmp3 = usart3_rx[0];

        if (u3state < 4) {
            if (tmp3 == 0xAA) {
                Rxbuffer_3[u3state] = tmp3;
                u3state++;
            } else {
                u3state = 0;
            }
        } else if (u3state < 194) {
            Rxbuffer_3[u3state] = tmp3;
            u3state++;
            crc3 += tmp3;
        } else if (u3state == 194) {
            Rxbuffer_3[u3state] = tmp3;
            if (tmp3 == crc3 % 256) {
                UartFlag[2] = 1;
                usart3_frame_cnt++;
            }
            u3state = 0;
            crc3    = 0;
        }
        HAL_UART_Receive_IT(&huart3, usart3_rx, 1);
    }

    if (huart->Instance == USART6) {
        static char   pi_line[32];
        static uint8_t pi_idx = 0;
        char c = (char)usart6_rx[0];

        if (c == '\n' || c == '\r') {
            if (pi_idx > 0) {
                pi_line[pi_idx] = '\0';
                if (pi_line[0] == 'D') {
                    // 数字串: D12345
                    strncpy(pi_digit_str, pi_line, sizeof(pi_digit_str) - 1);
                    pi_digit_str[sizeof(pi_digit_str) - 1] = '\0';
                    pi_digit_ready = 1;
                } else if (pi_line[0] == 'B') {
                    // 豆子码: 0/1/2/3
                    pi_bean_code = pi_line[1];
                    pi_bean_ready = 1;
                }
            }
            pi_idx = 0;  // 无论是否有效行，换行后重置
        } else if (pi_idx < sizeof(pi_line) - 1) {
            pi_line[pi_idx++] = c;
        } else {
            pi_idx = 0;  // 溢出保护：丢弃脏数据并重置，防止永久卡死
        }

        HAL_UART_Receive_IT(&huart6, usart6_rx, 1);
    }

}
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *uartHandle)
// {
//     uartHandle->RxState = HAL_UART_STATE_READY;
//     if (uartHandle->Instance == USART2) {
//         HAL_UART_Receive_IT(&huart2, (Rxbuffer[1]), 195);
        
//     }
// }


/*************************树莓派数据解码**********************************/
// uint16_t Upper_Target_Decode()
// {
//     static union {
//         uint8_t data[20];
//         float weight_state[5];
//     } state;
//     uint16_t success_cnt=0;

//     for (uint16_t i = 0; i < 3000;i++)

//         if ((receive_buffer[0] == 0xFF) && (receive_buffer[1] == 0xFE) && (receive_buffer[22] == 0xFE) && (receive_buffer[23] == 0xFF)) {
//             for (int i = 0; i < 20; i++) {
//                 state.data[i] = receive_buffer[i + 2];
//             }

//             for (int t = 0; t < 5; t++) {
//                 weight_placement[t] = state.weight_state[t];
//             }
//             success_cnt = 1;
//         }
//     return success_cnt;
// }

/*************************树莓派安全发送（不干扰RX中断）*************************/
void Pi_SendString(const char *str)
{
    if (str == NULL) return;
    // 暂禁 RXNE 中断，防止 HAL 状态冲突
    __HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
    while (*str) {
        while (!(USART6->SR & USART_SR_TXE));  // 等待发送寄存器空
        USART6->DR = (uint8_t)(*str++);
    }
    while (!(USART6->SR & USART_SR_TC));       // 等待发送完成
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

//另一个版本的解码(int类型)
// void Upper_Target_Decode()
// {
//     if ((receive_buffer[0] == 0xFF) && (receive_buffer[1] == 0xFE) && (receive_buffer[22] == 0xFE) && (receive_buffer[23] == 0xFF)) {
//         for (int i = 0; i < 5; i++) {
//             weight_placement[i] = receive_buffer[2 + 4 * i] + receive_buffer[2 + 4 * i + 1] << 8 + receive_buffer[2 + 4 * i + 2] << 16 + receive_buffer[2 + 4 * i + 3] << 24;
//         }
//     }
// }

