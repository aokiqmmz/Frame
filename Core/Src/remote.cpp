//
// Created by h on 2024/12/6.
//

#include "remote.h"
#include <cstdint>

Remote::Remote() {
    // 初始化数据
    channel_ = {0.0f, 0.0f, 0.0f, 0.0f};
    switch_ = {up, up};
    mouse_ = {0.0f, 0.0f, 0.0f, unpressed, unpressed};
}

// 线性映射函数
float Remote::linearMapping(int value, int in_min, int in_max, float out_min, float out_max) {
    return (float)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Remote::RemoteDataProcess(const uint8_t *pData) {
    if (pData == nullptr)
        return;

    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                          ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;

    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];

    // 线性映射到 RCChannel
    channel_.r_row = linearMapping(RC_CtrlData.rc.ch0, 364, 1684, -1, 1);
    channel_.r_col = linearMapping(RC_CtrlData.rc.ch1, 364, 1684, -1, 1);
    channel_.l_row = linearMapping(RC_CtrlData.rc.ch2, 364, 1684, -1, 1);
    channel_.l_col = linearMapping(RC_CtrlData.rc.ch3, 364, 1684, -1, 1);

    // 更新 switch 状态
    switch_.l = (RCSwitchState_e)(RC_CtrlData.rc.s1 - 1);
    switch_.r = (RCSwitchState_e)(RC_CtrlData.rc.s2 - 1);

    // 更新 mouse 数据
    mouse_.x = linearMapping(RC_CtrlData.mouse.x, -32768, 32768, -1, 1);
    mouse_.y = linearMapping(RC_CtrlData.mouse.y, -32768, 32768, -1, 1);
    mouse_.z = linearMapping(RC_CtrlData.mouse.z, -32768, 32768, -1, 1);
    mouse_.l_mouse_press = (RCMousePress)(RC_CtrlData.mouse.press_l);
    mouse_.r_mouse_press = (RCMousePress)(RC_CtrlData.mouse.press_r);
}

Remote remote;