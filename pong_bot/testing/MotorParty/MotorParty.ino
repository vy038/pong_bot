// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
#include <Servo.h> 



// DC motor on M2
AF_DCMotor motorRB(1);
AF_DCMotor motorRF(2);
AF_DCMotor motorLF(3);
AF_DCMotor motorLB(4);
// DC hobby servo
Servo servo1;


unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor party!");
  
  // turn on servo
  servo1.attach(10);
   
  // turn on motor #2
  motorRB.setSpeed(200);
  motorRF.setSpeed(200);
  motorLB.setSpeed(200);
  motorLF.setSpeed(200);
  motorRB.run(RELEASE);
  motorRF.run(RELEASE);
  motorLB.run(RELEASE);
  motorLF.run(RELEASE);

  startMillis = millis();  //initial start time
}

int i;

// Test the DC motor, stepper and servo ALL AT ONCE!
void loop() {
  
  motorRF.run(FORWARD);
}

