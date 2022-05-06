/* 
   差分轮PID控制头文件
   Functions and type-defs for PID control.

   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* 电机的PID设定信息 PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // 目标速度(编码器计数) target speed in ticks per frame
  long Encoder;                  // 编码器计数 encoder count
  long PrevEnc;                  // 编码器前次计数 last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // 最后输入 last input
  //int PrevErr;                   // last error

  /*
  * 使用积分项（ITerm）代替积分误差（Ierror） Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  double diamete_ratio;         //yanjingang:左轮相对于右轮轮径比系数(255饱和输出直行，往哪侧偏，则将对侧轮调大)
  long output;                  // 最后电机设置 last motor setting
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* PID参数 PID Parameters */
int Kp = 20;//15;
int Kd = 12;
int Ki = 0;
int Ko = 50;

unsigned char moving = 0; // 底盘是否在执行移动 is the base in motion?

/*
* 将PID变量初始化为零以防止启动峰值 Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both Encoder and PrevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.diamete_ratio = 1.035972814; //1.035972814; //yanjingang:左轮相对右轮的饱和输出时的轮速差比，需要反复试，确保pwm255饱和输出时能走直线不偏向一侧（>1.0表示此轮较快，需减慢）
   leftPID.output = 0;
   leftPID.PrevInput = 0;
   leftPID.ITerm = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.diamete_ratio = 1.0;
   rightPID.output = 0;
   rightPID.PrevInput = 0;
   rightPID.ITerm = 0;
}


/* 用于计算下一个电机指令的PID例程 PID routine to compute the next motor commands */
// 左右电机具体PID调试函数
void doPID(SetPointInfo * p) {
  long Perror;
  long output;
  int input;

  //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;

  //debug
  /*Serial.print(p->TargetTicksPerFrame);
  Serial.print(" ");
  Serial.print(p->Encoder);
  Serial.print("-");
  Serial.print(p->PrevEnc);
  Serial.print("=");
  //根据 input 绘图(工具-串口绘图器)
  Serial.println(input);*/

  
  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  //Serial.println(output);
  // 累积积分误差*或*限制输出 Accumulate Integral error *or* Limit output.
  // 当输出饱和时停止累积 Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM / p->diamete_ratio;  // 输出饱和时，处理左右电机的轮速差
  else if (output <= -MAX_PWM)
    output = -MAX_PWM / p->diamete_ratio;  // 输出饱和时，处理左右电机的轮速差
  else
    //允许转弯更改 allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}


/* 读取编码器值->计算PID->设置电机PWM Read the encoder values and call the PID routine */
void updatePID() {
  /* 读取编码器 Read the encoders */
  leftPID.Encoder = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);
  
  /* moving为0时，停止移动并重置PID参数  If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * 重置PIDs一次，以防止启动峰值 Reset PIDs once, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * PrevInput被认为是一个很好的检测代理 PrevInput is considered a good proxy to detect
    * 置是否已经发生 whether reset has already happened
    */
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) resetPID();
    return;
  }

  /* 计算每个电机的PID更新 Compute PID update for each motor */
  doPID(&leftPID);  //调试时，可以先调试单个电机的PID。比如，可以先注释 doPID(&rightPID)；
  doPID(&rightPID); 

  /* 相应地设置电机转速 Set the motor speeds accordingly */
  /*Serial.print("speed left: ");
  Serial.print(leftPID.output);
  Serial.print(" right: ");
  Serial.println(rightPID.output);*/
  setMotorSpeeds(leftPID.output, rightPID.output);
}
