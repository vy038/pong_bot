// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
#include <Servo.h>

// DC motors
AF_DCMotor motorRB(1);
AF_DCMotor motorRF(2);
AF_DCMotor motorLF(3);
AF_DCMotor motorLB(4);

Servo servo;

int leftIRSensor = 15;
int backIRSensor = 16;
int rightIRSensor = 17;

int motorWind1 = 18;
int motorWind2 = 19;

unsigned long startMillisFiring;  //some global variables available anywhere in the program
unsigned long currentMillisFiring;
int stage = 0;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;

int decision;
int fireOrNo = 1;

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps

  pinMode(motorWind1, OUTPUT);
  pinMode(motorWind2, OUTPUT);

  pinMode(leftIRSensor, INPUT);
  pinMode(backIRSensor, INPUT);
  pinMode(rightIRSensor, INPUT);


  servo.attach(9);
  servo.write(180);

  motorRB.setSpeed(100);
  motorRB.run(RELEASE);
  motorRF.run(RELEASE);
  motorLF.run(RELEASE);
  motorLB.run(RELEASE);

  randomSeed(analogRead(0));

  startMillisFiring = millis();
  startMillis = millis();
}

void loop() {
  if (IRRead() == 0) {
    //code for random movement and firing here

    currentMillis = millis();                               //get the current "time" (actually the number of milliseconds since the program started)
    if (currentMillis - startMillis >= random(2000, 4000))  //test whether the period has elapsed, adjust milisecond delay (2000) as needed
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
      forward(1);
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
      motorRB.run(RELEASE);
      motorRF.run(RELEASE);
      motorLF.run(RELEASE);
      motorLB.run(RELEASE);
      fire();
    }




  } else {
    Serial.println("Edging");
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
    case 0:
      motorRB.run(BACKWARD);
      motorRF.run(BACKWARD);
      motorLF.run(BACKWARD);
      motorLB.run(BACKWARD);
      break;
    case 1:
      motorRB.run(FORWARD);
      motorRF.run(FORWARD);
      motorLF.run(FORWARD);
      motorLB.run(FORWARD);
      break;
  }
}

void side(int dir) {
  switch (dir) {
    case 0:  //to the right
      motorRB.run(BACKWARD);
      motorRF.run(BACKWARD);
      motorLF.run(FORWARD);
      motorLB.run(FORWARD);
      break;
    case 1:  //to the left
      motorRB.run(BACKWARD);
      motorRF.run(BACKWARD);
      motorLF.run(FORWARD);
      motorLB.run(FORWARD);
      break;
  }
}

void fire() {  //fires a ball, works fine, adjust timing
  while (true) {

    if (stage == 0) {  //STAGE 1: turn motor pulling firing pin back for 500 ms, move firing trigger into position
      servo.write(180);
      Serial.println("0+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      digitalWrite(motorWind1, HIGH);
      digitalWrite(motorWind2, LOW);
    } 
    
    currentMillisFiring = millis();                      //get the current "time" (actually the number of milliseconds since the program started)
    if (currentMillisFiring - startMillisFiring >= 500)  //test whether the period has elapsed, adjust milisecond delay (500) as needed
    {
      stage += 1;
      startMillisFiring = currentMillisFiring;  //IMPORTANT to save the start time of the current time
    }

    if (stage == 1) {  //STAGE 2: turn motor with string back to original position for 500 ms
      Serial.println("1+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      digitalWrite(motorWind1, LOW);
      digitalWrite(motorWind2, HIGH);
    } else if (stage == 2) {  //STAGE 3: stop motor at original position, trigger pushes pin firing the ball in 500 ms
      Serial.println("2+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      digitalWrite(motorWind1, LOW);
      digitalWrite(motorWind2, LOW);
      servo.write(0);
    } else {  //reset stage after 500 ms grace period
      stage = 0;
      Serial.println("Fire end+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      fireOrNo = 1;
      break;
    }
  }
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