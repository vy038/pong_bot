// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>



// DC motors
AF_DCMotor motorRB(1);
AF_DCMotor motorRF(2);
AF_DCMotor motorLF(3);
AF_DCMotor motorLB(4);


void setup() {
  Serial.begin(9600);
  Serial.println("testing");
  
  motorRB.setSpeed(90);
  motorRF.setSpeed(90);
  motorLB.setSpeed(90);
  motorLF.setSpeed(90);
  motorRB.run(RELEASE);
  motorRF.run(RELEASE);
  motorLB.run(RELEASE);
  motorLF.run(RELEASE);

}

void loop() {
  forward(1);
}


void forward(int dir) {
  switch (dir) {
    case 0: //backwards
      motorRB.run(BACKWARD);
      motorRF.run(BACKWARD);
      motorLF.run(BACKWARD);
      motorLB.run(BACKWARD);
      break;
    case 1: //forwards
      motorRB.run(FORWARD);
      motorRF.run(FORWARD);
      motorLF.run(FORWARD);
      motorLB.run(FORWARD);
      break;
  }
}
