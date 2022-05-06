/***************************************************************
   电机驱动头文件，马达驱动板都要实现此文件定义的三个函数
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef  QGP_MOTOR_DRIVER    // QGP MotorShield
  // 控制引脚
  #define LEFT_MOTOR_PIN    1  // Motor 1
  #define RIGHT_MOTOR_PIN   2  // Motor 2

#elif  L298P_MOTOR_DRIVER    // L298P
  // PWM调速引脚
  #define LEFT_MOTOR_PWM    10  // A motor PWM调速(控制转速)
  #define RIGHT_MOTOR_PWM   11  // B motor PWM调速(控制转速)

  // 普通数字引脚（控制方向）
  #define LEFT_MOTOR_DIR   12  // A motor 使能(控制转动方向)
  #define RIGHT_MOTOR_DIR  13  // B motor 使能(控制转动方向)


#elif defined L298N_MOTOR_DRIVER            // L298N
  // 左轮电机驱动写引脚（接L298电驱板input口）
  #define LEFT_MOTOR_FORWARD   9
  #define LEFT_MOTOR_BACKWARD  10
  
  // 右轮电机驱动写引脚（接L298电驱板input口）
  #define RIGHT_MOTOR_FORWARD  5
  #define RIGHT_MOTOR_BACKWARD 6

  // 电机开关（目前没用，可以不接）
  //#define RIGHT_MOTOR_ENABLE 12
  //#define LEFT_MOTOR_ENABLE 13

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
