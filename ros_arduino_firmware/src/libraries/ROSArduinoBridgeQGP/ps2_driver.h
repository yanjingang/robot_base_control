/**
 * @file ps2.h
 * @author yanjingang@mail.com
 * @brief ps2蓝牙手柄控制初始化
 *         注意：手柄接收器不支持热插拔，否则必须重启控制板
 * @version 0.1
 * @date 2022-01-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
 
#include "PS2X_lib.h"                     //调用库文件PS2X_lib，这个是无线手柄库

PS2X ps2x;                     //定义PS2手柄

int ps2_error = 0;      //连接错误
byte ps2_type = 0;      //手柄类型
byte ps2_vibrate = 0;   //震动
byte ps2_motor_spd = 60; //默认控制电机转速


//设置引脚 setup pins and settings:  GamePad(clock, command, attention,  data, Pressures?, Rumble?) 
#define PS2_ATTENTION_PIN  10 //定义PS2手柄引脚
#define PS2_COMMAND_PIN    11
#define PS2_DATA_PIN       12
#define PS2_CLOCK_PIN      13
#define PS2_USE_PRESSURES  false  // 启用手柄压感
#define PS2_USE_RUMBLE     true  // 启用手柄蜂鸣
