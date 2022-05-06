/***************************************************************
   电机驱动实现，初始化控制器，设置速度
   马达驱动实现代码，根据预定义选择不同的驱动板库，在这里我使用了L298P，所以需要自己实现一个新的驱动库，后面会介绍
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined QGP_MOTOR_DRIVER    // QGP MotorShield驱动版
  #include "QGPMaker_MotorShield.h"

  QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
  QGPMaker_DCMotor *leftMotor = AFMS.getMotor(LEFT_MOTOR_PIN);  // 1-4马达
  QGPMaker_DCMotor *rightMotor = AFMS.getMotor(RIGHT_MOTOR_PIN);

  // 初始化
  void initMotorController() {
    AFMS.begin(50); // create with the default frequency 50Hz
  }

  // 单电机运动
  void setMotorSpeed(int i, int spd) {
    uint8_t direction = FORWARD;

    if (spd < 0){   //反转
        spd = -spd;
        direction = BACKWARD;
    }
    if (spd > 255){  //过载控制
        spd = 255;
    }

    if (i == LEFT) { 
        leftMotor->setSpeed(spd); //设置电机速度(0 stopped ~ 255 full speed)
        leftMotor->run(direction);  //启动电机(FORWARD 正转, BACKWARD 反转 ,RELEASE 关闭电机）
    }else {
        rightMotor->setSpeed(spd);
        rightMotor->run(direction);  //启动电机(FORWARD 正转, BACKWARD 反转 ,RELEASE 关闭电机）
    }
  }
  // 多电机控制
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined L298P_MOTOR_DRIVER    // L298P电机驱动版

  // L298P固定转向引脚初始化
  void initMotorController() {
    pinMode(LEFT_MOTOR_DIR, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR, OUTPUT);
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  }
  // 单电机运动
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd < 0){   //反转
        spd = -spd;
        reverse = 1;
    }
    if (spd > 255)  //过载控制
        spd = 255;

    if (i == LEFT) { 
        if (reverse == 0) { 
            digitalWrite(LEFT_MOTOR_DIR, HIGH);  // 正转
        }else if (reverse == 1) { 
            digitalWrite(LEFT_MOTOR_DIR, LOW);   // 反转
        }
        analogWrite(LEFT_MOTOR_PWM, spd);  // PWM 调速
    }else /*if (i == RIGHT) //no need for condition*/ {
        if (reverse == 0) { 
            digitalWrite(RIGHT_MOTOR_DIR, LOW);   // 反转
        }else if (reverse == 1) { 
            digitalWrite(RIGHT_MOTOR_DIR, HIGH);  // 正转
        }
        analogWrite(RIGHT_MOTOR_PWM, spd);  // PWM 调速
    }
  }
  // 多电机控制
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined L298N_MOTOR_DRIVER    // L298N
  void initMotorController() {
    //digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    //digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == RIGHT) { 
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == LEFT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#else
  #error A motor driver must be selected!
#endif

#endif
