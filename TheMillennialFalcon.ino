/*************************************************************
  File:      FinalProject.ino
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

 /*---------------Module Defines-----------------------------*/
 /*----- States -----*/
#define  SEARCHING_FOR_FLAP    0
#define  GOING_TOWARDS_FLAP    1
#define  RELOADING_TOKENS      2
#define  SEARCHING_FOR_BEACON  3  
#define  GOING_TOWARDS_BEACON  4
#define  DUMPING_TOKENS        5
#define  SYSTEM_OFF            6

/*----- Timers -----*/
#define ROBOT_TIMER            0

/*----- Constants -----*/
#define  ONE_SECOND             1000
#define  TOTAL_TIME             120*ONE_SECOND  
#define  FLAP                   0
#define  BEACON                 1

/*---------------Module Function Prototypes---*/
void SearchingForFlap(void); // search for FLAP
void GoingTowardsFlap(void); // return for more tokens
void ReloadingTokens(void); // go to beacon
void SearchingForBeacon(void); // search for beacon
void GoingTowardsBeacon(void); // go to beacon
void DumpingTokens(void); // initialize servo
void SystemOff(void);

void StartTimer(int timer, int time);

void TurnRight(void);
void TurnLeft(void);
void GoForward(void);
void GoBackwards(void);
void Stop(void);
void DeployTokenDispenser(void);

unsigned char TestForFLAPOrBeacon(int pin);
unsigned char TestForCarryingTokens(void);
unsigned char TestForIfBumperHit(void);


 /*---------------Global Variables-------*/
 /*---- Arduino Pins-----*/
int IRPinLeft = 2; // IR pin at the left side
int IRPinRight = 4; // IR pin at the right side
int IRPinCenter = 7; // IR pin at the front side

 /*---- State Variables-----*/
boolean state = 0; // begin with 7 tokens

 /*----Other Variables-----*/
int frequency = 0; // initialize frequency

void setup() {
  Serial.begin(9600);
  
  // Initialize IR pins
  pinMode(IRPinLeft, INPUT);
  pinMode(IRPinRight, INPUT);
  pinMode(IRPinCenter, INPUT);
  
  // Start timer
  StartTimer(ROBOT_TIMER, TOTAL_TIME);  
}

void loop() {
  switch(state) {
    case(SEARCHING_FOR_FLAP): SearchingForFlap(); break;
    case(GOING_TOWARDS_FLAP): GoingTowardsFlap(); break;
    case(RELOADING_TOKENS): ReloadingTokens(); break;
    case(SEARCHING_FOR_BEACON): SearchingForBeacon(); break;
    case(GOING_TOWARDS_BEACON): GoingTowardsBeacon(); break;
    case(DUMPING_TOKENS): DumpingTokens(); break;
    case(SYSTEM_OFF): SystemOff(); break;    
    
    default: Serial.println("Something went horribly wrong");
  }
}

void SearchingForFlap(void) {
  if (TestForFLAPOrBeacon(IRPinCenter) == FLAP) {
    Stop();
    state = GOING_TOWARDS_FLAP;
  } else if (TestForFLAPOrBeacon(IRPinRight) == FLAP) {
    TurnRight();
  } else if (TestForFLAPOrBeacon(IRPinLeft) == FLAP) {
    TurnLeft();
  } else {
    TurnRight();
  }
}

void GoingTowardsFlap(void) {
  if (TestForIfBumperHit()) {
    Stop();
  } else if (TestForFLAPOrBeacon(IRPinCenter) == FLAP) {
     GoForward();
   } else {
      state = SEARCHING_FOR_FLAP; 
   }
}

void ReloadingTokens(void); // go to beacon
void SearchingForBeacon(void); // search for beacon
void GoingTowardsBeacon(void); // go to beacon
void DumpingTokens(void) {
}
void SystemOff(void);

void StartTimer(int timer, int time) {
  TMRArd_InitTimer(timer, time);  
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
