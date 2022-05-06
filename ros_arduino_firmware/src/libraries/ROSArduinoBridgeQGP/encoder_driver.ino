/* *************************************************************
   编码器驱动实现, 读取编码器数据，重置编码器等
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef QGP_ENC_COUNTER  // QGP驱动版电机编码器
  #include "QGPMaker_Encoder.h"
  
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  
  QGPMaker_Encoder leftEncoder(LEFT_ENC_PIN);
  QGPMaker_Encoder rightEncoder(RIGHT_ENC_PIN);
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    /*Serial.print("readEncoder: left=");
    Serial.print(leftEncoder.read());
    Serial.print(" right=");
    Serial.println(rightEncoder.read());*/
    if (i == LEFT){
      left_enc_pos = leftEncoder.read();   //encoder.read()获取编码器数值；encoder.getRPM()获取编码器电机对应的每分钟转速（RPM）
      return left_enc_pos;
    }else{
      right_enc_pos = rightEncoder.read() * -1; //yanjingang:电机编码器取反（即放在两侧的两个电机正负未对调，此时前进或后退，两电机的编码器转动方向是反的，需要修正下）
      return right_enc_pos;
    }
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }

#elif defined(ARDUINO_ENC_COUNTER)  // Arduino+L298P电机编码器
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
  	enc_last <<=2; //shift previous state two places
  	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
    static uint8_t enc_last=0;
          	
  	enc_last <<=2; //shift previous state two places
  	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    //if (i == LEFT) return left_enc_pos * -1;  //yanjingang:左电机编码器取反（即放在两侧的两个电机正负未对调，此时前进或后退，两电机的编码器转动方向是反的，需要修正下）
    //else return right_enc_pos;
    else return right_enc_pos * -1; //yanjingang:电机编码器取反（即放在两侧的两个电机正负未对调，此时前进或后退，两电机的编码器转动方向是反的，需要修正下）
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
  
#elif defined ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }

#elif defined ARDUINO_MY_COUNTER
  volatile long left_count = 0L;
  volatile long right_count = 0L;
  void initEncoders(){
    pinMode(LEFT_A, INPUT); // 21  --- 2
    pinMode(LEFT_B, INPUT); // 20  --- 3
    pinMode(RIGHT_A, INPUT);// 18  --- 5
    pinMode(RIGHT_B, INPUT);// 19  --- 4

    attachInterrupt(2, leftEncoderEventA, CHANGE);
    attachInterrupt(3, leftEncoderEventB, CHANGE);
    attachInterrupt(5, rightEncoderEventA, CHANGE);
    attachInterrupt(4, rightEncoderEventB, CHANGE);
  }
  void leftEncoderEventA(){
    if(digitalRead(LEFT_A) == HIGH){
      if(digitalRead(LEFT_B) == HIGH){
        left_count++;
      } else {
        left_count--;
      }
    } else {
      if(digitalRead(LEFT_B) == LOW){
        left_count++;
      } else {
        left_count--;
      }
    }
  }
  void leftEncoderEventB(){
    if(digitalRead(LEFT_B) == HIGH){
      if(digitalRead(LEFT_A) == LOW){
        left_count++;
      } else {
        left_count--;
      }
    } else {
      if(digitalRead(LEFT_A) == HIGH){
        left_count++;
      } else {
        left_count--;
      }
    }
  }
  void rightEncoderEventA(){
    if(digitalRead(RIGHT_A) == HIGH){
      if(digitalRead(RIGHT_B) == HIGH){
        right_count++;
      } else {
        right_count--;
      }
    } else {
      if(digitalRead(RIGHT_B) == LOW){
        right_count++;
      } else {
        right_count--;
      }
    }  
  }
  void rightEncoderEventB(){
     if(digitalRead(RIGHT_B) == HIGH){
      if(digitalRead(RIGHT_A) == LOW){
        right_count++;
      } else {
        right_count--;
      }
    } else {
      if(digitalRead(RIGHT_A) == HIGH){
        right_count++;
      } else {
        right_count--;
      }
    }  
  }

  long readEncoder(int i) {
    if (i == LEFT) return left_count;
    else return right_count;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_count=0L;
      return;
    } else { 
      right_count=0L;
      return;
    }
  }
  
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
