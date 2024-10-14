/*
Ping Pong Bot
Version 1.2
Victor Yu
Created: 2024-04-06
Modified: 2024-04-13

Purpose:
This is a robot that is intended to move around one side of a ping pong table 
(with distance sensors so it doesn't fall off), and fire ping pong balls at the user for training
*/

int leftIRSensor = 15;
int backIRSensor = 16;
int rightIRSensor = 17;


void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

void loop() {
  // Serial.print(digitalRead(A1));
  // Serial.print(" ");
  // Serial.print(digitalRead(A2));
  // Serial.print(" ");
  // Serial.println(digitalRead(A3));
  if (IRRead() == 1) { //left detected
    Serial.println("left detected");
  } else if (IRRead() == 2) { //right detected
    Serial.println("right detected");
  } else if (IRRead() == 3) { //back detected
    Serial.println("back detected");
  } else {
    Serial.println("N/A");
  }

  delay(10); 
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