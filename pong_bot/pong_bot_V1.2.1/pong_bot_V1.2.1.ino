/*
Ping Pong Bot
Version 1.3
Victor Yu
Created: 2024-04-06
Modified: 2024-08-25

Purpose:
This is a robot that is intended to move around one side of a ping pong table 
(with distance sensors so it doesn't fall off), and fire ping pong balls at the user for training
*/

#include <Adafruit_MotorShield.h>
#include <Servo.h>

//i2c address for shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// DC motors
Adafruit_DCMotor *motorRB = AFMS.getMotor(1);
Adafruit_DCMotor *motorRF = AFMS.getMotor(2);
Adafruit_DCMotor *motorLF = AFMS.getMotor(3);
Adafruit_DCMotor *motorLB = AFMS.getMotor(4);

Servo servoPull;
Servo servoWheel;

int leftIRSensor = 17;
int backIRSensor = 16;
int rightIRSensor = 15;

unsigned long startMillisFiring;  //some global variables available anywhere in the program
unsigned long currentMillisFiring;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;

int decision;
int fireOrNo = 1;

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  pinMode(leftIRSensor, INPUT);
  pinMode(backIRSensor, INPUT);
  pinMode(rightIRSensor, INPUT);


  servoPull.attach(9);
  servoPull.write(180);

  motorRB->setSpeed(140);
  motorRF->setSpeed(140);
  motorLF->setSpeed(140);
  motorLB->setSpeed(140);
  motorRB->run(RELEASE);
  motorRF->run(RELEASE);
  motorLF->run(RELEASE);
  motorLB->run(RELEASE);

  randomSeed(analogRead(0));

  servoWheel.attach(10);
  
  servoPull.write(180);
  delay(500);
  servoWheel.write(160);
  delay(950);
  servoWheel.write(20);
  delay(400);
  servoWheel.write(90);

  startMillisFiring = millis();
  startMillis = millis();
}

void loop() {
  if (IRRead() == 0) {
    //code for random movement and firing here

    currentMillis = millis();                               //get the current "time" (actually the number of milliseconds since the program started)
    if (currentMillis - startMillis >= random(2000, 4000))  //test whether the period has elapsed, adjust milisecond delay (2000 - 4000) as needed
    {
      decision = random(1, 5);
      if (fireOrNo != -1) {
        fireOrNo = random(1, 9);
        Serial.println("eibwefwewerwewrewreewrwrerewrewrewrewwrewrewrewer");
      }
      Serial.println(fireOrNo);
      startMillis = currentMillis;  //IMPORTANT to save the start time of the current time
    }

    if (decision == 1) {
      //forward(1);
      Serial.println("fwd");
    } else if (decision == 2) {
      forward(1);
      Serial.println("bck");
    } else if (decision == 3) {
      side(1);
      Serial.println("left");
    } else if (decision == 4) {
      side(0);
      Serial.println("right");
    }

    if (fireOrNo >= 7) {
      fireOrNo = -1;
      Serial.println("FIRE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      motorRB->run(RELEASE);
      motorRF->run(RELEASE);
      motorLF->run(RELEASE);
      motorLB->run(RELEASE);
      fire();
    }




  } else {
    Serial.println("Edging");
    Serial.print(digitalRead(A1));
    Serial.print(" ");
    Serial.print(digitalRead(A2));
    Serial.print(" ");
    Serial.println(digitalRead(A3));
    if (IRRead() == 1) {
      side(0);
      delay(random(500, 1500));
    } else if (IRRead() == 2) {
      side(1);
      delay(random(500, 1500));
    } else {
      forward(1);
      delay(random(500, 1500));
    }
  }

  delay(10); //save resources
}

void forward(int dir) {
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

int IRRead() {  //reads the sensors on the side
  if (digitalRead(leftIRSensor) == LOW && digitalRead(backIRSensor) == LOW && digitalRead(rightIRSensor) == LOW) {
    return 0;
  } else if (digitalRead(leftIRSensor) == HIGH) {
    return 1;
  } else if (digitalRead(rightIRSensor) == HIGH) {
    return 2;
  } else {
    return 3;
  }
}