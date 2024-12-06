//
// Created by h on 2024/12/6.
//

#include "control.h"
#include "pid.h"
#include "imu.h"
#include "stm32f4xx_hal.h" // include _hal_uart.h  fail
#include "usart.h"
#include "remote.h"
#include "can.h"
#include "motor.h"

uint8_t canData[8];
int refe;

void sendCanMessage(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *pData, uint8_t Length) {

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = StdId;
    TxHeader.ExtId = 0x00;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = Length;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, pData, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

void mainLoop() {
    //mainCnt++;
    //HAL_IWDG_Refresh(&hiwdg);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, remote.buffer, 18);

    imu.readGyroRate();
    imu.readAccelRate();
    imu.dataProcess(0.0004);

    if (remote.switch_.r == down) {
        uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        sendCanMessage(&hcan1, 0x1FF, data, 8);
    }
    else {
        pid_pitch_angle.PID_Calc(remote.channel_.l_col, imu.p_degree);
        pid_pitch_speed.PID_Calc(pid_pitch_angle.output, pit_motor.rotate_speed_);

        pid_yaw_angle.PID_Calc(refe, imu.p_degree);
        pid_yaw_speed.PID_Calc(pid_yaw_angle.output, pit_motor.rotate_speed_);

        uint16_t scaled_output_pit = (uint16_t)(pid_pitch_speed.output) + (-34.5156*imu.p_degree + 1.7781e+03);
        uint16_t scaled_output_yaw = (uint16_t)(pid_yaw_speed.output);

        canData[0] = scaled_output_pit >> 8 & 0xFF;
        canData[1] = scaled_output_pit & 0xFF;
        canData[4] = scaled_output_yaw >> 8 & 0xFF;
        canData[5] = scaled_output_yaw & 0xFF;
        sendCanMessage(&hcan1, 0x1FF, canData, 8);
        }
};

