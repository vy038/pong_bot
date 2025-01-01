/*
Ping Pong Bot (RC Version)
Version 1.1
Victor Yu
Created: 2024-08-28
Modified: 2024-08-31

Purpose:
Based on Ping Pong Bot 1.3 (Auto), this is adapted for a wireless controller
*/

// libraries
#include <Adafruit_MotorShield.h>
#include <Servo.h>

//i2c address for shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// DC motors definition
Adafruit_DCMotor *motorRB = AFMS.getMotor(1);
Adafruit_DCMotor *motorRF = AFMS.getMotor(2);
Adafruit_DCMotor *motorLF = AFMS.getMotor(3);
Adafruit_DCMotor *motorLB = AFMS.getMotor(4);

// define servos
Servo servoPull;
Servo servoWheel;

// pin definitions of sensors
const int leftIRSensor = 6;
const int backIRSensor = 7;
const int rightIRSensor = 12;

// define where us sensor is 
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
float duration;
long frontDistance;

// controller variables
const int buttonA = 4; 
const int buttonB = 5; 
const uint8_t backward = A2; 
const uint8_t forward = A1; 
const uint8_t left = A0;
const uint8_t right = A3;

// logic variables for the robot to decide when to fire
int fireOrNo = 1;

// timer stuff (variables)
unsigned long startMillisFiring;  //some global variables available anywhere in the program
unsigned long currentMillisFiring;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;

void setup() {
  // serial monitor
  Serial.begin(9600);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // configure sensor pin modes
  pinMode(leftIRSensor, INPUT);
  pinMode(backIRSensor, INPUT);
  pinMode(rightIRSensor, INPUT);

  // configure pins for distance sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // input pins for controller
  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);
  pinMode(forward, INPUT);
  pinMode(backward, INPUT);
  pinMode(left, INPUT);
  pinMode(right, INPUT);

  // set up the trigger servo
  servoPull.attach(9);
  servoPull.write(180);

  // configure wheel servo
  servoWheel.attach(10);

  // configure motors
  motorRB->setSpeed(140);
  motorRF->setSpeed(140);
  motorLF->setSpeed(140);
  motorLB->setSpeed(140);
  motorRB->run(RELEASE);
  motorRF->run(RELEASE);
  motorLF->run(RELEASE);
  motorLB->run(RELEASE);

  // set random seed for robot decision making
  randomSeed(analogRead(0));
  
  // arm the robot
  servoPull.write(180);
  delay(500);
  servoWheel.write(160); // *NOTE* CHANGE THE ANGLE OF ROTATION WHEN INSTALLING NEW SERVO WHEEL
  delay(950);
  servoWheel.write(20);
  delay(400);
  servoWheel.write(90);

  // start timers
  startMillisFiring = millis();
  startMillis = millis();
}

void loop() {
  if (IRRead() == 0) { // if all sensors detect that the bot is not on an edge or near a wall
    // code for controlled movement and firing here

    if (digitalRead(left) == HIGH) {
      side(0);
    } else if (digitalRead(right) == HIGH) {
      side(1);
    } else if (digitalRead(backward) == HIGH) {
      forwardFunc(0);
    } else if (digitalRead(forward) == HIGH) {
      forwardFunc(1);
    } else {
      motorRB->run(RELEASE);
      motorRF->run(RELEASE);
      motorLF->run(RELEASE);
      motorLB->run(RELEASE);
    }

    if (digitalRead(buttonA) == HIGH) {
      fire();
    }


  } else { // if bot is on the edge of something
    Serial.println("Edging");
    Serial.print(digitalRead(A1));
    Serial.print(" ");
    Serial.print(digitalRead(A2));
    Serial.print(" ");
    Serial.println(digitalRead(A3));

    // takes cases form the IRRead() function and moves the bot accordingly (moves away from edge or wall)
    if (IRRead() == 1) {
      side(0);
      delay(500);
    } else if (IRRead() == 2) {
      side(1);
      delay(500);
    } else if (IRRead() == 3) {
      forwardFunc(1);
      delay(500);
    } else {
      forwardFunc(0);
      delay(500);
    }
  }

  delay(10); //save resources
}

// move forward function (or backwards, depending on input variable)
void forwardFunc(int dir) {
  switch (dir) {
    case 0: //backwards
      motorRB->run(BACKWARD);
      motorRF->run(BACKWARD);
      motorLF->run(BACKWARD);
      motorLB->run(BACKWARD);
      break;
    case 1: //forwards
      motorRB->run(FORWARD);
      motorRF->run(FORWARD);
      motorLF->run(FORWARD);
      motorLB->run(FORWARD);
      break;
  }
}

// move left or right depending in input variable (0 is right, 1 is left)
void side(int dir) {
  switch (dir) {
    case 0:  //to the right
      motorRB->run(BACKWARD);
      motorRF->run(FORWARD);
      motorLF->run(BACKWARD);
      motorLB->run(FORWARD);
      break;
    case 1:  //to the left
      motorRB->run(FORWARD);
      motorRF->run(BACKWARD);
      motorLF->run(FORWARD);
      motorLB->run(BACKWARD);
      break;
  }
}



// firing procedure (CONFIGURE ROTATION AMOUNT TO ACCOUNT FOR MG995 WHEEL)
void fire() {  //fires a ball, works fine, adjust timing (Maybe replace with milis()?)
  //STAGE 1: pull firing pin, firing ball
  servoPull.write(0);
  Serial.println("0+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  delay(500);

  //STAGE 2: turn motor with string back to original position for 750 ms
  Serial.println("1+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  servoPull.write(180);
  delay(500);

  Serial.println("1.5++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"); 
  servoWheel.write(160);
  delay(950);

  //STAGE 3: return motor to og pos
  Serial.println("2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  servoWheel.write(20);
  delay(400);
  
  //reset stage after 500 ms grace period
  Serial.println("Fire end+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  servoWheel.write(90);
  fireOrNo = 1;
}

/* reads the sensors on the side and distance sensor at the front
  * returns 0: all is well
  * returns 1: left edge
  * returns 2: right edge
  * returns 3: back edge
  * returns 4: front wall
*/
int IRRead() {  
  /************ Start US Measurement Section ***********/
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  frontDistance = duration * 0.034/2;
  /************** End US Measurement Section ***********/

  if (digitalRead(leftIRSensor) == LOW && digitalRead(backIRSensor) == LOW && digitalRead(rightIRSensor) == LOW && frontDistance > 10) {
    return 0;
  } else if (digitalRead(leftIRSensor) == HIGH) {
    return 1;
  } else if (digitalRead(rightIRSensor) == HIGH) {
    return 2;
  } else if (digitalRead(backIRSensor) == HIGH) {
    return 3;
  } else {
    return 4;
  }
}