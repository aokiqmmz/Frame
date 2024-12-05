//
// Created by h on 2024/12/6.
//
#include "callback.h"
#include <main.h>
#include <cstring>
#include <usart.h>
#include "motor.h"
#include "remote.h"
#include "control.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        mainLoop();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        memcpy(remote.data, remote.buffer, 18);
        remote.RemoteDataProcess(remote.data);
        HAL_UART_Receive_DMA(&huart3, remote.buffer, 18);
    }
}


void canRxMsgCallback(uint8_t rx_data[8], Motor *motor) {
    motor->updateMotorStatus(rx_data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rxData[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rxData) == HAL_OK) {

            if (rx_header.StdId == CAN_MOTOR_PIT_ID) {
                canRxMsgCallback(rxData, &pit_motor);
            } else if (rx_header.StdId == CAN_MOTOR_YAW_ID) {
                canRxMsgCallback(rxData, &yaw_motor);
            } else {

            }
        }
    }
}



