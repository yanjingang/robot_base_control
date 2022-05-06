/******************************************************************
  It will only work with http://www.7gp.cn
 ******************************************************************/

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>

#include "QGPMaker_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"

#if defined(ARDUINO_SAM_DUE)
#define WIRE Wire1
#else
#define WIRE Wire
#endif

#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

QGPMaker_MotorShield::QGPMaker_MotorShield(uint8_t addr)
{
  _addr = addr;
  _pwm = Adafruit_MS_PWMServoDriver(_addr);
}

void QGPMaker_MotorShield::begin(uint16_t freq)
{
  // init PWM w/_freq
  WIRE.begin();
  _pwm.begin();
  _freq = freq;
  _pwm.setPWMFreq(_freq); // This is the maximum PWM frequency
  for (uint8_t i = 0; i < 16; i++)
    _pwm.setPWM(i, 0, 0);
}

void QGPMaker_MotorShield::setPWM(uint8_t pin, uint16_t value)
{
  if (value > 4095)
  {
    _pwm.setPWM(pin, 4096, 0);
  }
  else
    _pwm.setPWM(pin, 0, value);
}
void QGPMaker_MotorShield::setPin(uint8_t pin, boolean value)
{
  if (value == LOW)
    _pwm.setPWM(pin, 0, 0);
  else
    _pwm.setPWM(pin, 4096, 0);
}

QGPMaker_DCMotor *QGPMaker_MotorShield::getMotor(uint8_t num)
{
  if (num > 4)
    return NULL;

  num--;

  if (dcmotors[num].motornum == 0)
  {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t pwm, in1, in2;
    if (num == 0)
    {
      pwm = 8;
      in2 = 9;
      in1 = 8;
    }
    else if (num == 1)
    {
      pwm = 13;
      in2 = 11;
      in1 = 10;
    }
    else if (num == 2)
    {
      pwm = 2;
      in2 = 14;
      in1 = 15;
    }
    else if (num == 3)
    {
      pwm = 7;
      in2 = 12;
      in1 = 13;
    }
    //    dcmotors[num].PWMpin = pwm;
    dcmotors[num].IN1pin = in1;
    dcmotors[num].IN2pin = in2;
  }
  return &dcmotors[num];
}

QGPMaker_StepperMotor *QGPMaker_MotorShield::getStepper(uint16_t steps, uint8_t num)
{
  if (num > 2)
    return NULL;

  num--;

  if (steppers[num].steppernum == 0)
  {
    // not init'd yet!
    steppers[num].steppernum = num;
    steppers[num].revsteps = steps;
    steppers[num].MC = this;
    uint8_t pwma, pwmb, ain1, ain2, bin1, bin2;
    if (num == 0)
    {
      ain2 = 8;
      ain1 = 9;
      bin2 = 11;
      bin1 = 10;
    }
    else if (num == 1)
    {
      ain2 = 12;
      ain1 = 13;
      bin2 = 14;
      bin1 = 15;
    }
    steppers[num].AIN1pin = ain1;
    steppers[num].AIN2pin = ain2;
    steppers[num].BIN1pin = bin1;
    steppers[num].BIN2pin = bin2;
  }
  return &steppers[num];
}

QGPMaker_Servo *QGPMaker_MotorShield::getServo(uint8_t num)
{
  if (num > 7)
    return NULL;
  //num--;
  if (servos[num].servonum == 0)
  {
    // not init'd yet!
    servos[num].servonum = num;
    servos[num].MC = this;
    servos[num].PWMpin = num;
    servos[num].PWMfreq = _freq;
  }
  return &servos[num];
}

/******************************************
               SERVOS
******************************************/

QGPMaker_Servo::QGPMaker_Servo(void)
{
  MC = NULL;
  servonum = 0;
  PWMpin = 0;
  currentAngle = 0;
}

void QGPMaker_Servo::setServoPulse(double pulse)
{
  double pulselength;
  pulselength = 1000000; // 1,000,000 us per second
  pulselength /= 50;     // 50 Hz
  pulselength /= 4096;   // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  MC->setPWM(PWMpin, pulse);
}
void QGPMaker_Servo::writeServo(uint8_t angle)
{
  double pulse;
  pulse = 0.5 + angle / 90.0;
  setServoPulse(pulse);
  currentAngle = angle;
  /* if(n>1){
     currentAngle[n-12]=angle;
    }else{
     currentAngle[n]=angle;
    }*/
}

uint8_t QGPMaker_Servo::readDegrees()
{
  return currentAngle;
}

/******************************************
               MOTORS
******************************************/

QGPMaker_DCMotor::QGPMaker_DCMotor(void)
{
  MC = NULL;
  motornum = 0;
  _speed = IN1pin = IN2pin = 0;
}

void QGPMaker_DCMotor::run(uint8_t cmd)
{
  MDIR = cmd;
  switch (cmd)
  {
  case FORWARD:
    MC->setPin(IN2pin, LOW); // take low first to avoid 'break'
    MC->setPWM(IN1pin, _speed * 16);
    break;
  case BACKWARD:
    MC->setPin(IN1pin, LOW); // take low first to avoid 'break'
    MC->setPWM(IN2pin, _speed * 16);
    break;
  case RELEASE:
    MC->setPin(IN1pin, LOW);
    MC->setPin(IN2pin, LOW);
    break;
  case BRAKE:
    MC->setPin(IN1pin, HIGH);
    MC->setPin(IN2pin, HIGH);
    break;
  }
}

void QGPMaker_DCMotor::setSpeed(uint8_t speed)
{
  _speed = speed;
  run(MDIR);
}

/******************************************
               STEPPERS
******************************************/

QGPMaker_StepperMotor::QGPMaker_StepperMotor(void)
{
  revsteps = steppernum = currentstep = 0;
}
/*

  uint16_t steps, Adafruit_MotorShield controller)  {

  revsteps = steps;
  steppernum = 1;
  currentstep = 0;

  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0

    // enable both H bridges
    pinMode(11, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(11, HIGH);
    digitalWrite(3, HIGH);

    // use PWM for microstepping support
    MC->setPWM(1, 255);
    MC->setPWM(2, 255);

  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0

    // enable both H bridges
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);

    // use PWM for microstepping support
    // use PWM for microstepping support
    MC->setPWM(3, 255);
    MC->setPWM(4, 255);
  }
  }
*/

void QGPMaker_StepperMotor::setSpeed(uint16_t rpm)
{
  //Serial.println("steps per rev: "); Serial.println(revsteps);
  //Serial.println("RPM: "); Serial.println(rpm);

  usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
  //   Serial.print("steps per rev: "); Serial.println(revsteps);
  // Serial.print("usperstep: "); Serial.println(usperstep);
}

void QGPMaker_StepperMotor::release(void)
{
  MC->setPin(AIN1pin, LOW);
  MC->setPin(AIN2pin, LOW);
  MC->setPin(BIN1pin, LOW);
  MC->setPin(BIN2pin, LOW);
  // MC->setPWM(PWMApin, 0);
  // MC->setPWM(PWMBpin, 0);
}

void QGPMaker_StepperMotor::step(uint16_t steps, uint8_t dir, uint8_t style)
{
  uint32_t uspers = usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE)
  {
    uspers /= 2;
  }
  else if (style == MICROSTEP)
  {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = ");
    Serial.println(steps, DEC);
#endif
  }

  // while (steps--)
  // {
  //   unsigned long now = micros();
  //   // Serial.println("step!"); Serial.println(uspers);
  //   //  if (now - this->last_step_time >= uspers)
  //   // {
  //   //   this->last_step_time = now;
  //   //    ret = onestep(dir, style);
  //   // }
  //   ret = onestep(dir, style);
  //   delayMicroseconds(uspers); //uspers
  //   // yield(); // required for ESP8266
  // }

  while (steps > 0)
  {
    
    unsigned long now = micros();
     if (now - this->last_step_time >= uspers)
    {
      this->last_step_time = now;
       ret = onestep(dir, style);
       steps--;
    }
  }
}

uint8_t QGPMaker_StepperMotor::onestep(uint8_t dir, uint8_t style)
{
  uint8_t a, b, c, d;
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;

  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE)
  {
    if ((currentstep / (MICROSTEPS / 2)) % 2)
    { // we're at an odd step, weird
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS / 2;
      }
      else
      {
        currentstep -= MICROSTEPS / 2;
      }
    }
    else
    { // go to the next even step
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS;
      }
      else
      {
        currentstep -= MICROSTEPS;
      }
    }
  }
  else if (style == DOUBLE)
  {
    if (!(currentstep / (MICROSTEPS / 2) % 2))
    { // we're at an even step, weird
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS / 2;
      }
      else
      {
        currentstep -= MICROSTEPS / 2;
      }
    }
    else
    { // go to the next odd step
      if (dir == FORWARD)
      {
        currentstep += MICROSTEPS;
      }
      else
      {
        currentstep -= MICROSTEPS;
      }
    }
  }
  else if (style == INTERLEAVE)
  {
    if (dir == FORWARD)
    {
      currentstep += MICROSTEPS / 2;
    }
    else
    {
      currentstep -= MICROSTEPS / 2;
    }
  }

  if (style == MICROSTEP)
  {
    if (dir == FORWARD)
    {
      currentstep++;
    }
    else
    {
      // BACKWARDS
      currentstep--;
    }

    currentstep += MICROSTEPS * 4;
    currentstep %= MICROSTEPS * 4;

    ocra = ocrb = 0;
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
    {
      ocra = microstepcurve[MICROSTEPS - currentstep];
      ocrb = microstepcurve[currentstep];
    }
    else if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
    {
      ocra = microstepcurve[currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS * 2 - currentstep];
    }
    else if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
    {
      ocra = microstepcurve[MICROSTEPS * 3 - currentstep];
      ocrb = microstepcurve[currentstep - MICROSTEPS * 2];
    }
    else if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
    {
      ocra = microstepcurve[currentstep - MICROSTEPS * 3];
      ocrb = microstepcurve[MICROSTEPS * 4 - currentstep];
    }
  }

  currentstep += MICROSTEPS * 4;
  currentstep %= MICROSTEPS * 4;

#ifdef MOTORDEBUG
  Serial.print("current step: ");
  Serial.println(currentstep, DEC);
  Serial.print(" pwmA = ");
  Serial.print(ocra, DEC);
  Serial.print(" pwmB = ");
  Serial.println(ocrb, DEC);
#endif
  //TODO 速度控制
  // MC->setPWM(PWMApin, ocra * 16);
  // MC->setPWM(PWMBpin, ocrb * 16);

  // release all
  uint8_t latch_state = 0; // all motor pins to 0

  //Serial.println(step, DEC);
  if (style == MICROSTEP)
  {
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
      latch_state |= 0x03;
    if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
      latch_state |= 0x06;
    if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
      latch_state |= 0x0C;
    if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
      latch_state |= 0x09;
  }
  else
  {
    // Serial.print("currentstep: ");
    // Serial.println(currentstep);
    switch (currentstep / (MICROSTEPS / 2))
    {
    case 0:
      latch_state |= 0x1; // energize coil 1 only
      break;
    case 1:
      latch_state |= 0x3; // energize coil 1+2
      break;
    case 2:
      latch_state |= 0x2; // energize coil 2 only
      break;
    case 3:
      latch_state |= 0x6; // energize coil 2+3
      break;
    case 4:
      latch_state |= 0x4; // energize coil 3 only
      break;
    case 5:
      latch_state |= 0xC; // energize coil 3+4
      break;
    case 6:
      latch_state |= 0x8; // energize coil 4 only
      break;
    case 7:
      latch_state |= 0x9; // energize coil 1+4
      break;
    }
  }
#ifdef MOTORDEBUG
  Serial.print("Latch: 0x");
  Serial.println(latch_state, HEX);
#endif
  // Serial.print("Latch: 0x");
  // Serial.println(latch_state, HEX);
  // Serial.print(">:");
  if (latch_state & 0x1)
  {
    MC->setPWM(AIN2pin, ocra * 16);
  }
  else
  {
    MC->setPin(AIN2pin, LOW);
  }
  if (latch_state & 0x2)
  {
    MC->setPWM(BIN1pin, ocrb * 16);
  }
  else
  {
    MC->setPin(BIN1pin, LOW);
  }
  if (latch_state & 0x4)
  {
    MC->setPWM(AIN1pin, ocra * 16);
  }
  else
  {
    MC->setPin(AIN1pin, LOW);
  }
  if (latch_state & 0x8)
  {
    MC->setPWM(BIN2pin, ocrb * 16);
  }
  else
  {
    MC->setPin(BIN2pin, LOW);
  }
  Serial.println(" ");
  return currentstep;
}
