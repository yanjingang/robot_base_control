/* 
    串口命令预定义
    Define single-letter commands that will be sent by the PC over the serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'  // 获取模拟引脚上的读数（例如a 3）
#define GET_BAUDRATE   'b'  // 获取波特率
#define PIN_MODE       'c'  // 设置引脚模式（0 input, 1 output）
#define DIGITAL_READ   'd'  // 获取数字引脚上的读数（例如d 4）
#define READ_ENCODERS  'e'  // 读取电机编码器计数
#define MOTOR_SPEEDS   'm'  // 指定速度进行移动（例如m 20 20）
#define PING           'p'  // 
#define RESET_ENCODERS 'r'  // 重置编码器计数
#define SERVO_WRITE    's'  // 设置舵机的位置 -弧度(0-3.14)
#define SERVO_READ     't'  // 读取舵机的位置
#define UPDATE_PID     'u'  // 更新PID
#define DIGITAL_WRITE  'w'  // 向数字引脚发送低(0)或高(1)信号
#define ANALOG_WRITE   'x'  // 向模拟引脚发送PWM模拟信号
#define LEFT            0   
#define RIGHT           1

#endif
