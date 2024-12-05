//
// Created by h on 2024/12/6.
//
#ifndef REMOTE_H
#define REMOTE_H

#endif // REMOTE_H

#include <cstdint>

typedef struct
{
    struct
    {
        uint16_t ch0;
        uint16_t ch1;
        uint16_t ch2;
        uint16_t ch3;
        uint8_t s1;
        uint8_t s2;
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;
} RC_Ctl_t;

typedef struct {
    float r_row;
    float r_col;
    float l_row;
    float l_col;
} RCChannel;

typedef enum {
    up,
    down,
    mid,
} RCSwitchState_e;

typedef struct {
    RCSwitchState_e l;
    RCSwitchState_e r;
} RCSwitch;

typedef enum {
    unpressed,
    pressed,
} RCMousePress;

typedef struct {
    float x;
    float y;
    float z;
    RCMousePress l_mouse_press;
    RCMousePress r_mouse_press;
} RCMouse;

class Remote {
public:
    uint8_t buffer[18];
    uint8_t data[18];

    RCChannel channel_;
    RCSwitch switch_;
    RCMouse mouse_;
    RC_Ctl_t RC_CtrlData;

    Remote();

    void RemoteDataProcess(const uint8_t *pData);
    float linearMapping(int value, int in_min, int in_max, float out_min, float out_max);
};

extern Remote remote;

