/**
 * @file ps2.h
 * @author yanjingang@mail.com
 * @brief ps2蓝牙手柄控制初始化
 * @version 0.1
 * @date 2022-01-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ps2x_lib.h"

// 初始化手柄控制对象
PS2X ps2x;

int ps2_error = 0;
byte ps2_type = 0;
byte ps2_vibrate = 0;
byte ps2_motor_spd = 50; //默认控制电机转速


//设置引脚 setup pins and settings:  GamePad(clock, command, attention,  data, Pressures?, Rumble?) 
#define PS2_ATTENTION_PIN  6
#define PS2_COMMAND_PIN    7
#define PS2_DATA_PIN       8
#define PS2_CLOCK_PIN      9
#define PS2_USE_PRESSURES  false  // 启用压感
#define PS2_USE_RUMBLE     true  // 启用蜂鸣
