/*********************************************************************
 *  ROSArduinoBridge 主程序
    核心功能实现，程序入口
 
    可以通过一组简单的串口命令来控制差分机器人并接收回传的传感器与里程计
    数据，默认使用的是 Arduino + L298P电机驱动模块，如果使用其他的
    编码器或电机驱动需要重写readEncoder()与setMotorSpeed()函数

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* 0.常量/变量初始化 */

//是否启用底盘控制器
#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
//启用基座控制器需要设置的电机驱动以及编码器驱动
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA
   
   /* 电机编码器信号处理 Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER
   
   /* 测试电机编码器信号处理 Encoders directly attached to Arduino board */
   //#define ARDUINO_MY_COUNTER

   /* L298P电机驱动版 Motor driver*/
   #define L298P_MOTOR_DRIVER
   
   /* L298N电机驱动版 Motor driver*/
   //#define L298N_MOTOR_DRIVER
   
#endif

//是否启用舵机
//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
//波特率
#define BAUDRATE     57600

/* Maximum PWM signal */
//最大PWM值
#define MAX_PWM        255

//根据Arduino型号来包含对应的头文件
#if defined(ARDUINO) && ARDUINO >= 100
   #include "Arduino.h"
#else
   #include "WProgram.h"
#endif

/* Include definition of serial commands */
//串口命令
#include "commands.h"

/* Sensor functions */
//传感器文件
#include "sensors.h"

/* Include servo support if required */
//如果启用舵机，需要包含的头文件
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

//如果启用基座控制器需要包含的头文件
#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"     //电机驱动

  /* Encoder driver function definitions */
  #include "encoder_driver.h"   //编码器驱动

  /* PID parameters and functions */
  #include "diff_controller.h"  //PID调速

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz 调速频率

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE; //调速周期(1/30秒)
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL; //PID调试的结束时刻标记

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 1000   //2000 每次move命令持续执行时间(可按需修改，调试PID时建议5000)
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif


// A pair of varibles to help parse serial commands (thanks Fergs)
// 帮助解析串行命令
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;           // 输入字符

// Variable to hold the current single-character command
char cmd;           // 当前的单字符命令

// Character arrays to hold the first and second arguments
char argv1[16];     // 两个参数字符
char argv2[16];

// The arguments converted to integers
long arg1;          // 两个参数整数
long arg2;


//是否启用PS2蓝牙手柄
//#define USE_PS2      // yanjingang: Enable the ps2x controller code
//#undef USE_PS2     // Disable the ps2x controller code

//如果启用PS2手柄，需要包含对应头文件
#ifdef USE_PS2
  #include "ps2_driver.h"  //yanjingang: ps2驱动及参数初始化
#endif


/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  // Initialize the motor controller if used */
  // 初始化底盘
  #ifdef USE_BASE
    #ifdef ARDUINO_ENC_COUNTER
        //set as inputs
        // 设置编码器引脚为输入
        DDRD &= ~(1<<LEFT_ENC_PIN_A);
        DDRD &= ~(1<<LEFT_ENC_PIN_B);
        DDRC &= ~(1<<RIGHT_ENC_PIN_A);
        DDRC &= ~(1<<RIGHT_ENC_PIN_B);
        
        //enable pull up resistors
        // 启用上拉电阻器
        PORTD |= (1<<LEFT_ENC_PIN_A);
        PORTD |= (1<<LEFT_ENC_PIN_B);
        PORTC |= (1<<RIGHT_ENC_PIN_A);
        PORTC |= (1<<RIGHT_ENC_PIN_B);
        
        // tell pin change mask to listen to left encoder pins
        // 告诉引脚更改掩码以侦听左侧编码器引脚
        PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
        // tell pin change mask to listen to right encoder pins
        // 告诉引脚更改掩码以侦听听右编码器管脚
        PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
        
        // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
        // 在通用中断掩码中启用PCINT1和PCINT2中断
        PCICR |= (1 << PCIE1) | (1 << PCIE2);
        
    #elif defined ARDUINO_MY_COUNTER
        initEncoders();
    #endif

    // 电机控制初始化（motor_driver.ino里对应的电驱板要实现次函数）
    initMotorController();
    // PID初始化（diff_controller.h）
    resetPID();
  #endif

  /* Attach servos if used */
  // 初始化舵机
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
        servos[i].initServo(
            servoPins[i],
            stepDelay[i],
            servoInitPosition[i]);
    }
  #endif

  // Initialize the ps2 controller if used */
  // 初始化PS2遥控器
  #ifdef USE_PS2
    // 设置PS2控制引脚
    Serial.print( "Search PS2 Controller.." );  //串口打印“搜索手柄中”
    int wait_ps2 = 500;  //搜索手柄时间
    do{
        //此函数为配置无线手柄的初始化通信能力，如果ok，返回值0
        ps2_error = ps2x.config_gamepad(PS2_CLOCK_PIN, PS2_COMMAND_PIN, PS2_ATTENTION_PIN, PS2_DATA_PIN, PS2_USE_PRESSURES, PS2_USE_RUMBLE);  // setup pins and settings:  GamePad(clock, command, attention,  data, Pressures, Rumble) 
        if( ps2_error == 0 ){
            Serial.println( "\nConfigured successful " );  //连接成功
            break;
        }else{
            Serial.print( "." );  //等待连接，此时每0.1秒串口输出'.'用于提示功能 
            delay( 100 );
            wait_ps2 -= 100;
        }
    }while(wait_ps2 > 0);  //while循环直到手柄连接成功
    
    if( ps2_error != 0 ){
      Serial.print( "Search PS2 Fail! Please check USE_PS2!" );
    }else{
      // 读取PS2设备类型
      ps2_type = ps2x.readType();
      switch (ps2_type) {
          case 0:
              Serial.println("Wireless DualShock Controller found ");
              break;
          case 1:
              Serial.println("Wireless DualShock Controller found ");
              break;
          case 2:
              Serial.println("Unknown Controller type found ");
              break;
      }
      ps2x.read_gamepad( true, 200 );  //开机震动0.2秒用于提示手柄连接成功
    }
    delay( 500 );
    
  #endif
}


/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  //读取串口命令
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    // 命令以回车为结束符
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }else if (chr == ' ') {  // 使用空格分隔命令的各个部分 Use spaces to delimit parts of the command
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }else {
      if (arg == 0) {
        // The first arg is the single-letter command
        // 第一个参数是单字母命令
        cmd = chr;
      }else if (arg == 1) {
        // Subsequent arguments can be more than one character
        // 后续参数可以是多个字符
        argv1[index] = chr;
        index++;
      }else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }


  // 读取PS2遥控指令
  #ifdef USE_PS2
    if (ps2_error == 1)  // skip loop if no controller found
        return;
    
    // 读取手柄的控制信号
    if (ps2_type != 2) {   // DualShock Controller
        ps2x.read_gamepad(false, ps2_vibrate);  // read controller and set large motor to spin at 'ps2_vibrate' speed
        // 开始/选择键
        if (ps2x.Button(PSB_START))  
            Serial.println("Start is being held");
        if (ps2x.Button(PSB_SELECT))
            Serial.println("Select is being held");
        // 左侧上下左右键
        if (ps2x.Button(PSB_PAD_UP)) {  
            Serial.print("Up held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
            // 设置电机PID目标转速
            lastMotorCommand = millis();
            moving = 1;
            leftPID.TargetTicksPerFrame = ps2_motor_spd;
            rightPID.TargetTicksPerFrame = ps2_motor_spd;
        }
        if (ps2x.Button(PSB_PAD_RIGHT)) {
            Serial.print("Right held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
            // 设置电机PID目标转速
            lastMotorCommand = millis();
            moving = 1;
            leftPID.TargetTicksPerFrame = ps2_motor_spd/2;
            rightPID.TargetTicksPerFrame = ps2_motor_spd/2 * -1;
        }
        if (ps2x.Button(PSB_PAD_LEFT)) {
            Serial.print("LEFT held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
            // 设置电机PID目标转速
            lastMotorCommand = millis();
            moving = 1;
            leftPID.TargetTicksPerFrame = ps2_motor_spd/2 * -1;
            rightPID.TargetTicksPerFrame = ps2_motor_spd/2;
        }
        if (ps2x.Button(PSB_PAD_DOWN)) {
            Serial.print("DOWN held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
            // 设置电机PID目标转速
            lastMotorCommand = millis();
            moving = 1;
            leftPID.TargetTicksPerFrame = ps2_motor_spd * -1;
            rightPID.TargetTicksPerFrame = ps2_motor_spd * -1;
        }
        // 振动强度：按下蓝色（X）按钮的力度
        //ps2_vibrate = ps2x.Analog(PSAB_BLUE);  // this will set the large motor ps2_vibrate speed based on how hard you press the blue (X) button
        // 右侧4个按键
        if (ps2x.NewButtonState())  // will be TRUE if any button changes state  (on to off, or off to on)
        {
            if (ps2x.Button(PSB_GREEN)){  // 右侧三角形键
                Serial.println("Triangle pressed");
            }
        }
        if (ps2x.ButtonPressed(PSB_RED)){  // 右侧圆形键
            Serial.println("Circle just pressed");
        }
        if (ps2x.ButtonReleased(PSB_PINK)){  // 右侧矩形键
            Serial.println("Square just released");
        }
        if (ps2x.NewButtonState(PSB_BLUE)){  // 右侧X键
            Serial.println("X just changed");
        }
        // 当按下R键时，输出两个摇杆的当前值
        if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))  // print stick values if either is TRUE
        {
            Serial.print("Stick Values:");
            Serial.print(ps2x.Analog(PSS_LY), DEC);  // Left stick, Y axis. Other options: LX, RY, RX
            Serial.print(",");
            Serial.print(ps2x.Analog(PSS_LX), DEC);
            Serial.print(",");
            Serial.print(ps2x.Analog(PSS_RY), DEC);
            Serial.print(",");
            Serial.println(ps2x.Analog(PSS_RX), DEC);
        }
    }
    delay(50);
  #endif
  

  // If we are using base control, run a PID calculation at the appropriate intervals
  // 启动底盘控制时，执行PID控制
  #ifdef USE_BASE
    //如果当前时刻大于 nextPID,那么就执行PID调速，并在 nextPID 上自增一个PID调试周期
    if (millis() > nextPID) {
        // 读取编码器值->计算PID->设置电机PWM（diff_controller.h）
        updatePID();
        nextPID += PID_INTERVAL;
    }
    
    // Check to see if we have exceeded the auto-stop interval
    // 电机指令自动停止
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
        setMotorSpeeds(0, 0);
        moving = 0;
    }
  #endif


  // Sweep servos
  // 扫描伺候
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
        servos[i].doSweep();
    }
  #endif


}




/* Clear the current command parameters */
// 重置命令参数
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
// 执行串口命令
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
    case GET_BAUDRATE:  /* 获取波特率 */
        Serial.println(BAUDRATE);
        break;
    case ANALOG_READ:   /* 获取模拟引脚上的读数（例如a 3） */
        Serial.println(analogRead(arg1));
        break;
    case DIGITAL_READ:  /* 获取数字引脚上的读数（例如d 4） */
        Serial.println(digitalRead(arg1));
        break;
    case ANALOG_WRITE:  /* 向模拟引脚发送PWM模拟信号 */
        analogWrite(arg1, arg2);
        Serial.println("OK"); 
        break;
    case DIGITAL_WRITE: /* 向数字引脚发送低(0)或高(1)信号 */
        if (arg2 == 0) digitalWrite(arg1, LOW);
        else if (arg2 == 1) digitalWrite(arg1, HIGH);
        Serial.println("OK"); 
        break;
    case PIN_MODE:      /* 设置引脚模式（0 input, 1 output） */
        if (arg2 == 0) pinMode(arg1, INPUT);
        else if (arg2 == 1) pinMode(arg1, OUTPUT);
        Serial.println("OK");
        break;
    case PING:
        Serial.println(Ping(arg1));
        break;

    #ifdef USE_SERVOS
    case SERVO_WRITE:   /* 设置舵机的位置 -弧度(0-3.14) */
        servos[arg1].setTargetPosition(arg2);
        Serial.println("OK");
        break;
    case SERVO_READ:    /* 读取舵机的位置 */
        Serial.println(servos[arg1].getServo().read());
        break;
    #endif
        
    #ifdef USE_BASE
    case READ_ENCODERS:    /* 读取电机编码器计数 */
        Serial.print(readEncoder(LEFT));
        Serial.print(" ");
        Serial.println(readEncoder(RIGHT));
        break;
    case RESET_ENCODERS:   /* 重置编码器计数 */
        resetEncoders();
        resetPID();
        Serial.println("OK");
        break;
    case MOTOR_SPEEDS:      /* 指定速度进行移动（例如m 20 20）*/
        /* Reset the auto stop timer */
        lastMotorCommand = millis();
        if (arg1 == 0 && arg2 == 0) {
            setMotorSpeeds(0, 0);
            resetPID();
            moving = 0;
        }
        else moving = 1;    // 移动指令只能通过"m 0 0"指令 或 AUTO_STOP_INTERVAL到时 停止
        leftPID.TargetTicksPerFrame = arg1;     // 这个目标速度其实是编码器计数（base_control通过线/角速度标定，获得实际移动距离及速度 与编码器计数的关系，从而转换为合理的编码器计数值速度指令）
        rightPID.TargetTicksPerFrame = arg2;
        Serial.println("OK");   // base_control用"OK"来判断指令是否成功，所以不要改内容
        break;
    case UPDATE_PID:        /* 设置PID参数 */
        while ((str = strtok_r(p, ":", &p)) != '\0') {
            pid_args[i] = atoi(str);
            i++;
        }
        Kp = pid_args[0];
        Kd = pid_args[1];
        Ki = pid_args[2];
        Ko = pid_args[3];
        Serial.println("OK");
        break;
    #endif

    default:
        Serial.println("Invalid Command");
        break;
  }
}
