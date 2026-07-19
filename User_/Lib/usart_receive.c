
#include "STP_Decode.h"
#include "stm32f4xx_it.h"

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
       // HAL_UART_Transmit(&huart6, (uint8_t*)"received\r\n", 10, 50);
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
uint16_t Upper_Target_Decode()
{
    static union {
        uint8_t data[20];
        float weight_state[5];
    } state;
    uint16_t success_cnt=0;

    for (uint16_t i = 0; i < 3000;i++)

        if ((receive_buffer[0] == 0xFF) && (receive_buffer[1] == 0xFE) && (receive_buffer[22] == 0xFE) && (receive_buffer[23] == 0xFF)) {
            for (int i = 0; i < 20; i++) {
                state.data[i] = receive_buffer[i + 2];
            }

            for (int t = 0; t < 5; t++) {
                weight_placement[t] = state.weight_state[t];
            }
            success_cnt = 1;
        }
    return success_cnt;
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

