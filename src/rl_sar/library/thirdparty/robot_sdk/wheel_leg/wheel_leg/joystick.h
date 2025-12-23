/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef WHEEL_LEG_SDK_JOYSTICK_H
#define WHEEL_LEG_SDK_JOYSTICK_H

#include <stdint.h>
// 16b
typedef union {
    struct {
        uint8_t R1          :1;
        uint8_t L1          :1;
        uint8_t start       :1;
        uint8_t select      :1;
        uint8_t R2          :1;
        uint8_t L2          :1;
        uint8_t F1          :1;
        uint8_t F2          :1;
        uint8_t A           :1;
        uint8_t B           :1;
        uint8_t X           :1;
        uint8_t Y           :1;
        uint8_t up          :1;
        uint8_t right       :1;
        uint8_t down        :1;
        uint8_t left        :1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

// 40 Byte
typedef struct {
    uint8_t head[2];
    xKeySwitchUnion btn;
    uint8_t lx;
    uint8_t rx;
    uint8_t ry;
    uint8_t L2;
    uint8_t ly;

    uint8_t idle[31];
} xRockerBtnDataStruct;

#endif 
