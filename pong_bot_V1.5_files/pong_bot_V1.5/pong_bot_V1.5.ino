/*
Ping Pong Bot
Version 1.5
Victor Yu
Created: 2024-04-06
Modified: 2024-09-15

Purpose:
This is a robot that is intended to move around one side of a ping pong table 
(with distance sensors so it doesn't fall off), and fire ping pong balls at the user for training
This has two modes, auto and manual. The robot moves and fires autonomously in auto, and manual has everything being controlled by a controller.
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
const int frontIRSensor = 8;

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
int decision = 0;
int fireOrNo = 1;
int armed = 0;

// timer stuff (variables)
unsigned long startMillisFiring;  //some global variables available anywhere in the program
unsigned long currentMillisFiring;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;

// robot mode
int mode = 0;

void setup() {
  // serial monitor
  Serial.begin(9600);

  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
                        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }
  Serial.println("Motor Shield found.");

  // configure sensor pin modes
  pinMode(leftIRSensor, INPUT);
  pinMode(backIRSensor, INPUT);
  pinMode(rightIRSensor, INPUT);
  pinMode(frontIRSensor, INPUT);

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
  if (armed == 0) {
    servoPull.write(180);
    delay(500);
    servoWheel.write(130);
    delay(950);
    servoWheel.write(0);
    delay(400);
    armed = 1;
  }

  // redeclare values of variables for when mode changes
  decision = 0;
  fireOrNo = 1;
  currentMillisFiring = 0;
  currentMillis = 0;

  // start timers
  startMillisFiring = millis();
  startMillis = millis();

  while (true) {
    if (mode == 0) {        // MANUAL MODE
      if (IRRead() == 0) {  // if all sensors detect that the bot is not on an edge or near a wall
        // code for controlled movement and firing here

        if ((digitalRead(right) == 1 || digitalRead(left) == 1) && digitalRead(backward) == LOW && digitalRead(forward) == LOW) {  // movement for true side
          side(digitalRead(right), digitalRead(buttonB));
        } else if ((digitalRead(backward) == 1 || digitalRead(forward) == 1) && digitalRead(right) == LOW && digitalRead(left) == LOW) {  // movement for true forward/backward
          forwardFunc(digitalRead(forward));
        } else if ((digitalRead(backward) == 1 || digitalRead(forward) == 1) && (digitalRead(right) == 1 || digitalRead(left) == 1)) {  // diagonal
          if (digitalRead(buttonB) == HIGH) {               // turn or
            side(digitalRead(right), digitalRead(buttonB));
          } else {                                          // go diagonal
            diagonal(digitalRead(forward), digitalRead(left));
          }
        } else {  // stop the bot
          motorRB->run(RELEASE);
          motorRF->run(RELEASE);
          motorLF->run(RELEASE);
          motorLB->run(RELEASE);
        }

        if (digitalRead(buttonA) == HIGH && digitalRead(buttonB) == HIGH) {  // change modes and reset all timers
          motorRB->run(RELEASE);
          motorRF->run(RELEASE);
          motorLF->run(RELEASE);
          motorLB->run(RELEASE);
          delay(750);
          mode = 1;
          break;
        } else if (digitalRead(buttonA) == HIGH) {  // fire when a button is pressed
          fire();
        }


      } else {  // if bot is on the edge of something
        Serial.println("Edging");
        Serial.print(digitalRead(frontIRSensor));
        Serial.print(" ");
        Serial.print(digitalRead(backIRSensor));
        Serial.print(" ");
        Serial.print(digitalRead(leftIRSensor));
        Serial.print(" ");
        Serial.println(digitalRead(rightIRSensor));

        // takes cases form the IRRead() function and moves the bot accordingly (moves away from edge or wall)
        if (IRRead() == 1) {
          side(0, 0);
          delay(500);
        } else if (IRRead() == 2) {
          side(1, 0);
          delay(500);
        } else if (IRRead() == 3) {
          forwardFunc(1);
          delay(500);
        } else {
          forwardFunc(0);
          delay(500);
        }
      }

      delay(10);            // save resources
    } else {                // AUTO MODE
      if (IRRead() == 0) {  // if all sensors detect that the bot is not on an edge or near a wall
        // code for random movement and firing here

        currentMillis = millis();                               // get the current "time" (actually the number of milliseconds since the program started)
        if (currentMillis - startMillis >= random(2000, 4000))  // test whether the period has elapsed, adjust milisecond delay (2000 - 4000) as needed
        {                                                       // random movement decision
          decision = random(1, 5);
          if (fireOrNo != -1) {  // seperate random variable to see if the robot should fire a ball or not
            fireOrNo = random(1, 9);
            Serial.println("eibwefwewerwewrewreewrwrerewrewrewrewwrewrewrewer");
          }
          Serial.println(fireOrNo);
          startMillis = currentMillis;  // IMPORTANT to save the start time of the current time
        }

        // uses the random number generated at a random time to determine which direction to move in
        if (decision == 1) {
          forwardFunc(1);
          Serial.println("fwd");
        } else if (decision == 2) {
          forwardFunc(0);
          Serial.println("bck");
        } else if (decision == 3) {
          side(1, 0);
          Serial.println("left");
        } else if (decision == 4) {
          side(0, 0);
          Serial.println("right");
        }

        // uses a random number generated periodically to see whether the bot should stop and fire a ball randomly
        if (fireOrNo >= 7) {
          fireOrNo = -1;
          Serial.println("FIRE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          motorRB->run(RELEASE);
          motorRF->run(RELEASE);
          motorLF->run(RELEASE);
          motorLB->run(RELEASE);
          fire();
        }

        if (digitalRead(buttonA) == HIGH && digitalRead(buttonB) == HIGH) {  // change modes and reset all timers
          motorRB->run(RELEASE);
          motorRF->run(RELEASE);
          motorLF->run(RELEASE);
          motorLB->run(RELEASE);
          delay(750);
          mode = 0;
          break;
        }


      } else {  // if bot is on the edge of something
        Serial.println("Edging");
        Serial.print(digitalRead(frontIRSensor));
        Serial.print(" ");
        Serial.print(digitalRead(backIRSensor));
        Serial.print(" ");
        Serial.print(digitalRead(leftIRSensor));
        Serial.print(" ");
        Serial.println(digitalRead(rightIRSensor));

        // takes cases form the IRRead() function and moves the bot accordingly (moves away from edge or wall)
        if (IRRead() == 1) {
          side(0, 0);
          delay(random(500, 1000));
        } else if (IRRead() == 2) {
          side(1, 0);
          delay(random(500, 1000));
        } else if (IRRead() == 3) {
          forwardFunc(1);
          delay(random(500, 1000));
        } else {
          forwardFunc(0);
          delay(random(500, 1000));
        }
      }

      delay(10);  //save resources
    }
  }
}

void loop() {
  setup();
}

// move forward function (or backwards, depending on input variable)
void forwardFunc(int dir) {
  switch (dir) {
    case 0:  // backwards
      motorRB->run(BACKWARD);
      motorRF->run(BACKWARD);
      motorLF->run(BACKWARD);
      motorLB->run(BACKWARD);
      break;
    case 1:  // forwards
      motorRB->run(FORWARD);
      motorRF->run(FORWARD);
      motorLF->run(FORWARD);
      motorLB->run(FORWARD);
      break;
  }
}

// move left or right depending in input variable (0 is right, 1 is left)
void side(int dir, int buttonState) {
  if (buttonState == 1) {
    switch (dir) {
      case 0:  // to the right
        motorRB->run(BACKWARD);
        motorRF->run(BACKWARD);
        motorLF->run(FORWARD);
        motorLB->run(FORWARD);
        break;
      case 1:  // to the left
        motorRB->run(FORWARD);
        motorRF->run(FORWARD);
        motorLF->run(BACKWARD);
        motorLB->run(BACKWARD);
        break;
    }
  } else {
    switch (dir) {
      case 0:  // to the right
        motorRB->run(BACKWARD);
        motorRF->run(FORWARD);
        motorLF->run(BACKWARD);
        motorLB->run(FORWARD);
        break;
      case 1:  // to the left
        motorRB->run(FORWARD);
        motorRF->run(BACKWARD);
        motorLF->run(FORWARD);
        motorLB->run(BACKWARD);
        break;
    }
  }
}

// move the bot in diagonal directions, using only two inputs
void diagonal(int F, int L) {
  if (F == 1) {  // either FL or FR
    switch (L) {
      case 0:  // to the right (FR)
        motorRB->run(FORWARD);
        motorRF->run(RELEASE);
        motorLF->run(FORWARD);
        motorLB->run(RELEASE);
        break;
      case 1:  // to the left (FL)
        motorRB->run(RELEASE);
        motorRF->run(FORWARD);
        motorLF->run(RELEASE);
        motorLB->run(FORWARD);
        break;
    }
  } else {  // either BL or BR
    switch (L) {
      case 0:  // to the right (BR)
        motorRB->run(RELEASE);
        motorRF->run(BACKWARD);
        motorLF->run(RELEASE);
        motorLB->run(BACKWARD);
        break;
      case 1:  // to the left (BL)
        motorRB->run(BACKWARD);
        motorRF->run(RELEASE);
        motorLF->run(BACKWARD);
        motorLB->run(RELEASE);
        break;
    }
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
  servoWheel.write(130);
  delay(950);

  //STAGE 3: return motor to og pos
  Serial.println("2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
  servoWheel.write(0);
  delay(400);

  //reset stage after 500 ms grace period
  Serial.println("Fire end+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
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
  frontDistance = duration * 0.034 / 2;
  /************** End US Measurement Section ***********/

  if (digitalRead(leftIRSensor) == LOW && digitalRead(backIRSensor) == LOW && digitalRead(rightIRSensor) == LOW && digitalRead(frontIRSensor) == LOW && frontDistance > 10) {
    return 0;
  } else if (digitalRead(leftIRSensor) == HIGH) {
    return 1;
  } else if (digitalRead(rightIRSensor) == HIGH) {
    return 2;
  } else if (digitalRead(backIRSensor) == HIGH) {
    return 3;
  } else if (digitalRead(frontIRSensor) == HIGH){
    return 4;
  } else {
    return 5;
  }
}