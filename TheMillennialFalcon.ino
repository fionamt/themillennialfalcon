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
#define  SEARCHING_FOR_BALANCE      3  
#define  GOING_TOWARDS_BALANCE      4
#define  DUMPING_TOKENS_SERVO_UP   5
#define  DUMPING_TOKENS_SERVO_DOWN 6
#define  SYSTEM_OFF                7

/*----- Timers -----*/
#define ROBOT_TIMER            0

/*----- Constants -----*/
#define  ONE_SECOND             1000
#define  TOTAL_TIME             120*ONE_SECOND  
#define  FLAP                   0
#define  BALANCE                1
#define  FORWARD_SPEED          130
#define  TURN_SPEED             60

/*---------------Module Function Prototypes---*/
void SearchingForBeacon(int beacon); // search for FLAP
void GoingTowardsFlap(void); // return for more tokens
void ReloadingTokens(void); // stay still
void SearchingForBalance(void); // search for balance
void GoingTowardsBalance(void); // go to balance
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

unsigned char TestForFLAPOrBalance(int pin);
unsigned char TestForCarryingTokens(void);
unsigned char TestForBumperHit(int bumperPin);
unsigned char TestForBothBumpersHit(void);


 /*---------------Global Variables-------*/
 /*---- Arduino Pins-----*/
int IRPinLeft = 2; // IR pin at the left side
int IRPinRight = 4; // IR pin at the right side
int IRPinCenter = 7; // IR pin at the front side

int servoPin = 3;
int motorPinDirLeft = 8;
int motorPinEnLeft = 9;
int motorPinDirRight = 5;
int motorPinEnRight = 11;

int bumperPinRight = 6;
int bumperPinLeft = 12;
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
  pinMode(motorPinDirLeft, OUTPUT);
  pinMode(motorPinEnLeft, OUTPUT);
  pinMode(motorPinDirRight, OUTPUT);
  pinMode(motorPinEnRight, OUTPUT);
  
  // Start timer
  StartTimer(ROBOT_TIMER, TOTAL_TIME);  
}

void loop() {
  if (TestTimerExpired(ROBOT_TIMER)) {
    state = SYSTEM_OFF;
  }
  switch(state) {
    case(SEARCHING_FOR_FLAP): SearchingForBeacon(FLAP); break;
    case(GOING_TOWARDS_FLAP): GoingTowardsFlap(); break;
    case(RELOADING_TOKENS): ReloadingTokens(); break;
    case(SEARCHING_FOR_BALANCE): SearchingForBeacon(BALANCE); break;
    case(GOING_TOWARDS_BALANCE): GoingTowardsBalance(); break;
    case(DUMPING_TOKENS_SERVO_UP): DumpingTokensServoUp(); break;
    case(DUMPING_TOKENS_SERVO_DOWN): DumpingTokensServoDown(); break;
    case(SYSTEM_OFF): SystemOff(); break;    
    
    default: Serial.println("Something went horribly wrong");
  }
}

void SearchingForBeacon(int beacon) {
  if (TestForFLAPOrBalance(IRPinCenter) == beacon) {
    Stop();
    if (beacon == FLAP) {
      state = GOING_TOWARDS_FLAP;
    } else {
      state = GOING_TOWARDS_BALANCE;
    }
  } else if (TestForFLAPOrBalance(IRPinRight) == beacon) {
    TurnRight();
  } else if (TestForFLAPOrBalance(IRPinLeft) == beacon) {
    TurnLeft();
  } else {
    TurnRight();
  }
}

void GoingTowardsFlap(void) {
  if (TestForBumperHit(bumperPinRight) || TestForBumperHit(bumperPinLeft)) {
    Stop();
    state = RELOADING_TOKENS;
  } else if (TestForFLAPOrBalance(IRPinCenter) == FLAP) {
     GoForward();
   } else {
      state = SEARCHING_FOR_FLAP; 
   }
}

void ReloadingTokens(void) {
  if (TestForCarryingTokens()) {
    state = SEARCHING_FOR_BALANCE; 
  } else {
    Stop();
  }
}

void GoingTowardsBalance(void) {
  if (TestForBothBumpersHit()) {
    Stop();
    state = DUMPING_TOKENS_SERVO_UP;
  } else if (TestForBumperHit(bumperPinRight)) {
    //TODO: reverse and turn right
  } else if (TestForBumperHit(bumperPinLeft)) {
    //TODO: reverse and turn left
  } else if (TestForFLAPOrBalance(IRPinCenter) == BALANCE) {
     GoForward();
   } else {
      state = SEARCHING_FOR_BALANCE; 
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
  if (TestForCarryingTokens()) {
    state = DUMPING_TOKENS_SERVO_DOWN;
  } else {
    state = SEARCHING_FOR_FLAP;
  }
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

unsigned char TestForFLAPOrBalance(int pin) {
   // 50% duty cycle --> T = 2*t_pulse --> f = 500000/t_pulse
  frequency = 500000/pulseIn(pin, LOW);
  if ((frequency > 4000) && (frequency < 6000)) {
    Serial.println("I see the FLAP!");
    return FLAP;
  } else if ((frequency > 800) && (frequency < 1200)) {
    Serial.println("I see a balance!");
    return BALANCE;
  }
}

unsigned char TestForCarryingTokens(void) {
  return digitalRead(tokenPin);
}

unsigned char TestForBothBumpersHit(void) {
  return (digitalRead(bumperPinRight) && digitalRead(bumperPinLeft));
}

unsigned char TestForBumperHit(int bumperPin) {
  return digitalRead(bumperPin);
}

void TurnRight(void) {
  digitalWrite(motorPinDirRight, LOW);
  digitalWrite(motorPinDirLeft, HIGH);
  analogWrite(motorPinEnLeft, TURN_SPEED);
  analogWrite(motorPinEnRight, TURN_SPEED);
}

void TurnLeft(void) {
  digitalWrite(motorPinDirRight, HIGH);
  digitalWrite(motorPinDirLeft, LOW);
  analogWrite(motorPinEnLeft, TURN_SPEED);
  analogWrite(motorPinEnRight, TURN_SPEED);
}

void GoForward(void) {
  digitalWrite(motorPinDirRight, HIGH);
  digitalWrite(motorPinDirLeft, HIGH);
  analogWrite(motorPinEnLeft, FORWARD_SPEED);
  analogWrite(motorPinEnRight, FORWARD_SPEED);
}

void GoBackwards(void) {
  digitalWrite(motorPinDirRight, LOW);
  digitalWrite(motorPinDirLeft, LOW);
  analogWrite(motorPinEnLeft, FORWARD_SPEED);
  analogWrite(motorPinEnRight, FORWARD_SPEED);
}

void Stop(void) {
  analogWrite(motorPinEnLeft, 0);
  analogWrite(motorPinEnRight, FORWARD_SPEED);
}
