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

/*----- Time Constants -----*/
#define  ONE_SECOND             1000
#define  TOTAL_TIME             120*ONE_SECOND  
#define  REVERSE_TIME           500
#define  TURN_TIME              500

/*----- Speed Constants -----*/
#define  FORWARD_LEFT_SPEED     145
#define  FORWARD_RIGHT_SPEED    100
#define  TURN_SPEED             100

/*----- Servo Constants -----*/
#define  SERVO_DOWN             120
#define  SERVO_UP               0

/*----- Other Constants -----*/
#define  FLAP                   1
#define  BALANCE                2
#define  NOTHING                0

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
//unsigned char TestForCarryingTokens(void);
unsigned char TestForBumperHit(int bumperPin);
unsigned char TestForBothBumpersHit(int direction);

 /*---------------Global Variables-------*/
 /*---- Arduino Pins-----*/
int IRPinBack = 4; // IR pin at the left side
int IRPinFront = 2; // IR pin at the right side

int servoPin = 11;
int motorPinDirLeft = 15;
int motorPinEnLeft = 5;
int motorPinDirRight = 8;
int motorPinEnRight = 3;

int bumperPinTopRight = 6;
int bumperPinTopLeft = 12;
int bumperPinBottomRight = 7;
int bumperPinBottomLeft = 14;
int tokenPin = 13;

 /*---- State Variables-----*/
int state = SEARCHING_FOR_FLAP; // begin with 7 tokens

 /*----Other Variables-----*/
int frequency = 0; // initialize frequency
Servo servo;
int counter = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize IR pins
  pinMode(IRPinFront, INPUT);
  pinMode(IRPinBack, INPUT);
  
  // Initialize limit switches
  pinMode(bumperPinTopRight, INPUT);
  pinMode(bumperPinTopLeft, INPUT);
  pinMode(bumperPinBottomRight, INPUT);
  pinMode(bumperPinBottomLeft, INPUT);
  pinMode(tokenPin, INPUT);
  
  // Initialize motors
  servo.attach(servoPin);
  servo.write(SERVO_UP);
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
  if (beacon == FLAP) {
    Serial.println("Searching for Flap!");
    if (TestForFLAPOrBalance(IRPinBack) == beacon) {
      Stop();
      state = GOING_TOWARDS_FLAP;
      return;
    }
  } else if (beacon == BALANCE) {
    Serial.println("Searching for balance!");
    if (TestForFLAPOrBalance(IRPinFront) == beacon) {
      Stop();
      state = GOING_TOWARDS_BALANCE;
      return;
    }
  } 
//  if (counter < 5) {
    TurnRight();
//    counter = counter + 1;
//  } else if (counter < 10) {
//    TurnLeft();
//    counter = counter + 1;
//  } else {
//    counter = 0;
//  }
}

void GoingTowardsFlap(void) {
  Serial.println("Going towards Flap!");
  if (TestForBumperHit(bumperPinBottomRight) || TestForBumperHit(bumperPinBottomLeft)) {
    Stop();
    state = RELOADING_TOKENS;
  } else if (TestForFLAPOrBalance(IRPinBack) == FLAP) {
     GoBackwards();
   } else {
      state = SEARCHING_FOR_FLAP; 
   }
}

void ReloadingTokens(void) {
  Serial.println("Reloading Tokens");
  Stop();
  delay(3000);
//  if (TestForCarryingTokens()) {
    state = SEARCHING_FOR_BALANCE; 
//  } else {
//    Stop();
//  }
}

void GoingTowardsBalance(void) {
  Serial.println("Going towards Balance");
  if (TestForBothBumpersHit(0)) {
    Stop();  
    state = DUMPING_TOKENS_SERVO_UP;
  } else if (TestForBumperHit(bumperPinTopRight)) {
    GoBackwards();
    delay(REVERSE_TIME);
    TurnRight();
    delay(TURN_TIME);
  } else if (TestForBumperHit(bumperPinTopLeft)) {
    delay(REVERSE_TIME);
    TurnLeft();
    delay(TURN_TIME);
  } else if (TestForFLAPOrBalance(IRPinFront) == BALANCE) {
     GoForward();
  } else {
     state = SEARCHING_FOR_BALANCE; 
  }
}

void DumpingTokensServoUp(void) {
  Serial.println("Dumping tokens servo up");
  servo.write(SERVO_DOWN);
  delay(1000);
  state = DUMPING_TOKENS_SERVO_DOWN;
}

void DumpingTokensServoDown(void) {
  Serial.println("Dumping tokens servo down");
  servo.write(SERVO_UP);
  delay(1000);
//  if (TestForCarryingTokens()) {
//    state = DUMPING_TOKENS_SERVO_DOWN;
//  } else {
    state = SEARCHING_FOR_FLAP;
//  }
}

void SystemOff(void) {
  Serial.println("System off!");
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
  Serial.println(frequency);
  if ((frequency > 4000) && (frequency < 6000)) {
    Serial.println("I see the FLAP!");
    return FLAP;
  } else if ((frequency > 800) && (frequency < 1200)) {
    Serial.println("I see a balance!");
    return BALANCE;
  } else {
    return NOTHING;
  }
}

//unsigned char TestForCarryingTokens(void) {
//  if (!digitalRead(tokenPin)) {
//    Serial.println("I have tokens!");
//  } else {
//     Serial.println("No tokens :("); 
//  }
//  return !digitalRead(tokenPin);
//}

// 0 is front, 1 is back
unsigned char TestForBothBumpersHit(int direction) {
  if (direction == 0) {
    return (digitalRead(bumperPinTopRight) && digitalRead(bumperPinTopLeft));  
  } else if (direction == 1) {
     return (digitalRead(bumperPinBottomRight) && digitalRead(bumperPinBottomLeft));  
  }
}

unsigned char TestForBumperHit(int bumperPin) {
  return digitalRead(bumperPin);
}

void TurnLeft(void) {
  digitalWrite(motorPinDirRight, LOW);
  digitalWrite(motorPinDirLeft, HIGH);
  analogWrite(motorPinEnLeft, TURN_SPEED);
  analogWrite(motorPinEnRight, TURN_SPEED);
}

void TurnRight(void) {
  digitalWrite(motorPinDirRight, HIGH);
  digitalWrite(motorPinDirLeft, LOW);
  analogWrite(motorPinEnLeft, TURN_SPEED);
  analogWrite(motorPinEnRight, TURN_SPEED);
}

void GoForward(void) {
  digitalWrite(motorPinDirRight, HIGH);
  digitalWrite(motorPinDirLeft, HIGH);
  analogWrite(motorPinEnLeft, FORWARD_LEFT_SPEED);
  analogWrite(motorPinEnRight, FORWARD_RIGHT_SPEED);
}

void GoBackwards(void) {
  digitalWrite(motorPinDirRight, LOW);
  digitalWrite(motorPinDirLeft, LOW);
  analogWrite(motorPinEnLeft, FORWARD_LEFT_SPEED);
  analogWrite(motorPinEnRight, FORWARD_RIGHT_SPEED);
}

void Stop(void) {
  analogWrite(motorPinEnLeft, 0);
  analogWrite(motorPinEnRight, 0);
}
