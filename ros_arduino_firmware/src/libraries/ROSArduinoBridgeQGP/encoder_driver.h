/* *************************************************************
   编码器驱动头文件
   这里只是针对了Arduino UNO，使用了中断接口D2,D3,和模拟接口A4，A5；所以电机编码器的输出信号线需要按照此规则接线，另外要注意编码器要有两路输出
   左侧电机的编码输出接D2,D3；右侧电机的编码输出接A4,A5
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   

#ifdef QGP_ENC_COUNTER
  // 电机编码器信号读端口
  #define LEFT_ENC_PIN 1  //M1
  #define RIGHT_ENC_PIN 2  //M2
  
#elif ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  // 左轮电机编码器信号读引脚（A motor 编码信号线接 D2/3）
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  // 右轮电机编码器信号读引脚（B motor 编码信号线连 A4/5）
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
  
#elif defined ARDUINO_MY_COUNTER
  #define LEFT_A 21   //--- 2
  #define LEFT_B 20   //--- 3
  #define RIGHT_A 18  //--- 5
  #define RIGHT_B 19  //--- 4
  void initEncoders();
  void leftEncoderEventA();
  void leftEncoderEventB();
  void rightEncoderEventA();
  void rightEncoderEventB();
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
