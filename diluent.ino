#include <TimerOne.h>
#include <Wire.h>
#include <Time.h>
#include <Adafruit_PWMServoDriver.h>


#define MOTOR_0        204
#define MOTOR_100      408
#define MOTOR_50       ((MOTOR_0+MOTOR_100)/2)


#define LED_1G    42
#define LED_2A    40
#define LED_3G    38
#define LED_4R    36
#define LED_5R    34
#define LED_6R    32
#define LED_ON(led)    digitalWrite(led, LOW)
#define LED_OFF(led)   digitalWrite(led, HIGH)
#define LED_ALLON()    LED_ON(LED_1G); LED_ON(LED_2A); LED_ON(LED_3G); LED_ON(LED_4R); LED_ON(LED_5R); LED_ON(LED_6R)
#define LED_ALLOFF()   LED_OFF(LED_1G); LED_OFF(LED_2A); LED_OFF(LED_3G); LED_OFF(LED_4R); LED_OFF(LED_5R); LED_OFF(LED_6R)
#define LED_ERROR(led) LED_OFF(LED_1G); LED_OFF(LED_2A); LED_OFF(LED_3G); LED_ON(led)

#define GREEN_TOGGLE         30

#define KNOB1         A8
#define KNOB2         A9
#define KNOB3         A10
#define KNOB4         A11


#define BELT_STEP             2
#define BELT_DIRECTION        5
#define BELT_ENCODER          A13

#define STATION_SEAL_MOTOR    14
#define STATION_SEAL_HALL     28
#define STATION_SEAL_HEAT     22
#define STATION_SEAL_TEMP     A12

#define STATION_FILL_SERVO    13
#define STATION_FILL_UP       300
#define STATION_FILL_DOWN     135

#define STATION_CUT_MOTOR     15
#define STATION_CUT_HALL      24


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

volatile double next_beltpos = 0;
volatile unsigned long belt_watchdog = 0;

volatile unsigned seal_temperature = 0;

volatile enum {
  INIT,
  MOVE_BELT,
  DONE,
  BELT_ERROR,
} mode = INIT;



void reset()
{
  //wdt_enable(WDTO_15MS);
  //while(1);
}

void setup()
{
  Serial.begin(9600);
  
  // Set LEDs to out (active low)
  pinMode(LED_1G, OUTPUT);
  pinMode(LED_2A, OUTPUT);
  pinMode(LED_3G, OUTPUT);
  pinMode(LED_4R, OUTPUT);
  pinMode(LED_5R, OUTPUT);
  pinMode(LED_6R, OUTPUT);
  LED_ALLON();
  
  // Set CNC shield ENable (active low)
  pinMode(8, OUTPUT);
  digitalWrite(8, 0);
  
  // Set up big green button
  pinMode(GREEN_TOGGLE, INPUT_PULLUP);
  
  // Set stepper pins
  pinMode(BELT_STEP, OUTPUT);
  pinMode(BELT_DIRECTION, OUTPUT);
  digitalWrite(BELT_DIRECTION, 0);
  
  // Init I2C PWM board to 50Hz
  pwm.begin();
  pwm.setPWMFreq(50);
  
  pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
  pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_DOWN);
  pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);

  delay(3000);
  LED_ALLOFF();
  
  // Wait for green button to depress
  LED_ON(LED_2A);
  while (digitalRead(GREEN_TOGGLE) == 0);
  
  // Initialize the cutter
  {
    pinMode(STATION_CUT_HALL, INPUT_PULLUP);
    
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_50);
    while (digitalRead(STATION_CUT_HALL) == 0);
    while (digitalRead(STATION_CUT_HALL) == 1);
    pwm.setPWM(STATION_CUT_MOTOR, 0, MOTOR_0);
  }
  
  // Initialize the filler
  {
    pwm.setPWM(STATION_FILL_SERVO, 0, STATION_FILL_UP);
    delay(250);
  }
  
  // Initialize sealer
  {
    pinMode(STATION_SEAL_HALL, INPUT_PULLUP);
    pinMode(STATION_SEAL_HEAT, OUTPUT);
    
    digitalWrite(STATION_SEAL_HEAT, HIGH);
    seal_temperature = analogRead(STATION_SEAL_TEMP);M
    
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_50);
    while (digitalRead(STATION_SEAL_HALL) == 0);
    while (digitalRead(STATION_SEAL_HALL) == 1);
    pwm.setPWM(STATION_SEAL_MOTOR, 0, MOTOR_0);
  }
  
  // Initialize belt
  {
    // Get encoder value
    next_beltpos = analogRead(BELT_ENCODER);
    next_beltpos += fmod(next_beltpos, 102.4);
  }
  
  // Configure stepper timer interrupt
  Timer1.initialize(300);
  Timer1.attachInterrupt(timerint);
  
  
  
  LED_OFF(LED_2A);
  mode = DONE;
}

void loop()
{
  while (digitalRead(GREEN_TOGGLE) == 1) {
    LED_OFF(LED_1G); LED_ON(LED_3G);
  }
  LED_OFF(LED_3G); LED_ON(LED_1G);
  //step1_max = map(analogRead(KNOB1), 0, 1023, 2, 50);
  //step2_max = map(analogRead(STEP2_KNOB), 0, 1023, 1, 100);
  ///step1_max = 25;
  //step1_max *= step1_max / 2;
  //step2_max *= step2_max;
  
  //int k1 = map(analogRead(STEP2_KNOB), 0, 1023, 204, 408);
  //int k2 = map(analogRead(STEP3_KNOB), 0, 1023, 204, 408);
  //int k3 = map(analogRead(STEP4_KNOB), 0, 1023, 204, 408);
  
  //pwm.setPWM(13, 0, k1);
  //pwm.setPWM(14, 0, k2);
  //pwm.setPWM(15, 0, k3);
  
  //Serial.print("K1 = "); Serial.print(k1);
  //Serial.print(", K2 = "); Serial.print(k2);
  //Serial.print(", K3 = "); Serial.print(k3);
  //Serial.println(".");
  
  //Serial.print("Encoder = "); Serial.print(analogRead(BELT_ENCODER)/10); Serial.println(".");
  //Serial.print("GREEN_TOGGLE = "); Serial.print(digitalRead(GREEN_TOGGLE)); Serial.println(".");
  //Serial.print("Knob1 = "); Serial.print(analogRead(KNOB1)); Serial.println(".");
  //Serial.print("Knob2 = "); Serial.print(analogRead(KNOB2)); Serial.println(".");
  //Serial.print("Knob3 = "); Serial.print(analogRead(KNOB3)); Serial.println(".");
  //Serial.print("Knob4 = "); Serial.print(analogRead(KNOB4)); Serial.println(".");
  //Serial.println(last_BELT_ENCODER);
  //delay(10);
  //delay(500);
  
  
  if (mode == DONE) {
    next_beltpos = fmod(next_beltpos + 102.4, 1024);
    belt_watchdog = millis();
    mode = MOVE_BELT;
    
    while (mode == MOVE_BELT);
    Serial.println(analogRead(BELT_ENCODER));
  }
  delay(1000);
  
  /*
  if (ledval == 0) {
    ledval = 1;
  }
  
  Serial.print("LED = "); Serial.print(ledval); Serial.println(".");

  digitalWrite(LED_1G, (ledval & (1 << 0)) ? 0 : 1);
  digitalWrite(LED_2A, (ledval & (1 << 1)) ? 0 : 1);
  digitalWrite(LED_3G, (ledval & (1 << 2)) ? 0 : 1);
  digitalWrite(LED_4R, (ledval & (1 << 3)) ? 0 : 1);
  digitalWrite(LED_5R, (ledval & (1 << 4)) ? 0 : 1);
  digitalWrite(LED_6R, (ledval & (1 << 5)) ? 0 : 1);

  ledval <<= 1;
  
  
  
  
  delay(500);
  */
}

void timerint()
{
  if (mode == MOVE_BELT || mode == INIT) {
    if(fabs((double)analogRead(BELT_ENCODER) - next_beltpos) > 10.0) {
      digitalWrite(BELT_STEP, digitalRead(BELT_STEP) ^ 1);
    } else {
      // Next step
      mode = DONE;
    }
  }
  
  /*
  unsigned BELT_ENCODER = analogRead(BELT_ENCODER);
  unsigned long last_BELT_ENCODER_wrapped = (last_BELT_ENCODER % 1024);
  
  if (BELT_ENCODER < last_BELT_ENCODER_wrapped) {
    last_BELT_ENCODER += 1024 - last_BELT_ENCODER_wrapped + BELT_ENCODER;
  } else {
    last_BELT_ENCODER += BELT_ENCODER - last_BELT_ENCODER_wrapped;
  }
  */
  
  /*
  if (++step2_val >= step2_max)
  {
    digitalWrite(STEP2_SIGPIN, digitalRead(STEP2_SIGPIN) ^ 1);
    step2_val = 0;
  }
  */
  
  // Check watchdogs
  if (mode == MOVE_BELT && (millis() - belt_watchdog) > 1000) {
    //mode = BELT_ERROR;
    LED_ERROR(LED_4R);
  }
}

