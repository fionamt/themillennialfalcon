/*************************************************************
  File:      TheMillennialFalcon.ino
  Contents:  
  Notes:     Target: Arduino Leonardo
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-02-22 FMT  program created
 ************************************************************/
 
 /*---------------Includes-----------------------------------*/
#include <Timers.h>
#include <Servo.h>

 /*---------------Module Defines-----------------------------*/
 /*----- States -----*/
#define  SEARCHING_FOR_FLAP        0
#define  GOING_TOWARDS_FLAP        1
#define  RELOADING_TOKENS          2
#define  SEARCHING_FOR_BEACON      3  
#define  GOING_TOWARDS_BEACON      4
#define  DUMPING_TOKENS_SERVO_UP   5
#define  DUMPING_TOKENS_SERVO_DOWN 6
#define  SYSTEM_OFF                7

/*----- Timers -----*/
#define ROBOT_TIMER            0

/*----- Constants -----*/
#define  ONE_SECOND             1000
#define  TOTAL_TIME             120*ONE_SECOND  
#define  FLAP                   0
#define  BEACON                 1

/*---------------Module Function Prototypes---*/
void SearchingForSensor(int sensor); // search for FLAP
void GoingTowardsFlap(void); // return for more tokens
void ReloadingTokens(void); // go to beacon
void SearchingForBeacon(void); // search for beacon
void GoingTowardsBeacon(void); // go to beacon
void DumpingTokensServoUp(void); // initialize servo
void DumpingTokensServoDown(void); // retract servo
void SystemOff(void);

void StartTimer(int timer, int time);
unsigned char TestTimerExpired(int timer);

void TurnRight(void);
void TurnLeft(void);
void GoForward(void);
void GoBackwards(void);
void Stop(void);
void DeployTokenDispenser(void);

unsigned char TestForFLAPOrBeacon(int pin);
unsigned char TestForCarryingTokens(void);
unsigned char TestForBumperHit(void);


 /*---------------Global Variables-------*/
 /*---- Arduino Pins-----*/
int IRPinLeft = 2; // IR pin at the left side
int IRPinRight = 4; // IR pin at the right side
int IRPinCenter = 7; // IR pin at the front side

int servoPin = 9;
int bumperPin = 12;
int tokenPin = 13;

 /*---- State Variables-----*/
boolean state = 0; // begin with 7 tokens

 /*----Other Variables-----*/
int frequency = 0; // initialize frequency
Servo servo;
int servoPos = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize IR pins
  pinMode(IRPinLeft, INPUT);
  pinMode(IRPinRight, INPUT);
  pinMode(IRPinCenter, INPUT);
  
  // Initialize motors
  servo.attach(servoPin);
  
  // Start timer
  StartTimer(ROBOT_TIMER, TOTAL_TIME);  
}

void loop() {
  if (TestTimerExpired(ROBOT_TIMER)) {
    state = SYSTEM_OFF;
  }
  switch(state) {
    case(SEARCHING_FOR_FLAP): SearchingForSensor(FLAP); break;
    case(GOING_TOWARDS_FLAP): GoingTowardsFlap(); break;
    case(RELOADING_TOKENS): ReloadingTokens(); break;
    case(SEARCHING_FOR_BEACON): SearchingForSensor(BEACON); break;
    case(GOING_TOWARDS_BEACON): GoingTowardsBeacon(); break;
    case(DUMPING_TOKENS_SERVO_UP): DumpingTokensServoUp(); break;
    case(DUMPING_TOKENS_SERVO_DOWN): DumpingTokensServoDown(); break;
    case(SYSTEM_OFF): SystemOff(); break;    
    
    default: Serial.println("Something went horribly wrong");
  }
}

void SearchingForSensor(int sensor) {
  if (TestForFLAPOrBeacon(IRPinCenter) == sensor) {
    Stop();
    if (sensor == FLAP) {
      state = GOING_TOWARDS_FLAP;
    } else {
      state = GOING_TOWARDS_BEACON;
    }
  } else if (TestForFLAPOrBeacon(IRPinRight) == sensor) {
    TurnRight();
  } else if (TestForFLAPOrBeacon(IRPinLeft) == sensor) {
    TurnLeft();
  } else {
    TurnRight();
  }
}

void GoingTowardsFlap(void) {
  if (TestForBumperHit()) {
    Stop();
    state = RELOADING_TOKENS;
  } else if (TestForFLAPOrBeacon(IRPinCenter) == FLAP) {
     GoForward();
   } else {
      state = SEARCHING_FOR_FLAP; 
   }
}

void ReloadingTokens(void) {
  if (TestForCarryingTokens()) {
    state = SEARCHING_FOR_BEACON; 
  } else {
    Stop();
  }
}

void GoingTowardsBeacon(void) {
  if (TestForBumperHit()) {
    Stop();
    state = DUMPING_TOKENS_SERVO_UP;
  } else if (TestForFLAPOrBeacon(IRPinCenter) == BEACON) {
     GoForward();
   } else {
      state = SEARCHING_FOR_BEACON; 
   }
}

void DumpingTokensServoUp(void) {
  servoPos = 180;
  servo.write(servoPos);
  delay(15);
  state = DUMPING_TOKENS_SERVO_DOWN;
}

void DumpingTokensServoDown(void) {
  servoPos = 0;
  servo.write(servoPos);
  delay(15);
  state = SEARCHING_FOR_FLAP;
}

void SystemOff(void) {
 Stop(); 
}

void StartTimer(int timer, int time) {
  TMRArd_InitTimer(timer, time);  
}

unsigned char TestTimerExpired(int timer) {
  return (unsigned char)(TMRArd_IsTimerExpired(timer));
}

unsigned char TestForFLAPOrBeacon(int pin) {
   // 50% duty cycle --> T = 2*t_pulse --> f = 500000/t_pulse
  frequency = 500000/pulseIn(pin, LOW);
  if ((frequency > 4000) && (frequency < 6000)) {
    Serial.println("I see the FLAP!");
    return FLAP;
  } else if ((frequency > 800) && (frequency < 1200)) {
    Serial.println("I see a balance!");
    return BEACON;
  }
}

unsigned char TestForCarryingTokens(void) {
  return digitalRead(tokenPin);
}

unsigned char TestForBumperHit(void) {
  return digitalRead(bumperPin);
}

void TurnRight(void) {
  Serial.println("Turning Right!");
}

void TurnLeft(void) {
  Serial.println("Turning Left!");
}

void GoForward(void) {
  Serial.println("Go Forward!");
}

void GoBackwards(void) {
  Serial.println("Go Backwards!");
}

void Stop(void) {
    Serial.println("Turn off motors!");
}
