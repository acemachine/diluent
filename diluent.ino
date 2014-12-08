#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


volatile unsigned int step1_max = 4;
unsigned int step1_val = 0;

volatile unsigned int step2_max = 4;
unsigned int step2_val = 0;

#define STEP1_SIGPIN    2
#define STEP1_DIRPIN    5
#define STEP1_DIRSET    0

#define STEP2_SIGPIN    3
#define STEP2_DIRPIN    6
#define STEP2_DIRSET    0


#define STEP1_KNOB   A8
#define STEP2_KNOB   A9

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup()
{
  // Set CNC shield ENable (active low)
  pinMode(8, OUTPUT);
  digitalWrite(8, 0);

  
  // Set stepper pins
  pinMode(STEP1_SIGPIN, OUTPUT);
  pinMode(STEP1_DIRPIN, OUTPUT);
  digitalWrite(STEP1_DIRPIN, STEP1_DIRSET);
  
  pinMode(STEP2_SIGPIN, OUTPUT);
  pinMode(STEP2_DIRPIN, OUTPUT);
  digitalWrite(STEP2_DIRPIN, STEP2_DIRSET);
  
  // Init I2C PWM board to 50Hz
  pwm.begin();
  pwm.setPWMFreq(50);
  
  Timer1.initialize(250);
  Timer1.attachInterrupt(stepperNext);
}

void loop()
{
  step1_max = map(analogRead(STEP1_KNOB), 0, 255, 1, 5);
  step2_max = map(analogRead(STEP2_KNOB), 0, 255, 1, 5);
  
  step1_max *= step1_max / 2;
  step2_max *= step2_max;
  
  delay(10);
}

void stepperNext()
{
  if (++step1_val >= step1_max)
  {
    digitalWrite(STEP1_SIGPIN, digitalRead(STEP1_SIGPIN) ^ 1);
    step1_val = 0;
  }
  
  if (++step2_val >= step2_max)
  {
    digitalWrite(STEP2_SIGPIN, digitalRead(STEP2_SIGPIN) ^ 1);
    step2_val = 0;
  }
}

