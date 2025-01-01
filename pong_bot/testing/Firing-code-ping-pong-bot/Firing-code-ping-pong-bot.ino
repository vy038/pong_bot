/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo servoPull;  // create servo object to control a servo
Servo servoWheel;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);

  servoPull.attach(9);  // attaches the servo on pin 9 to the servo object
  servoPull.write(180);

  servoWheel.attach(10);  // attaches the servo on pin 9 to the servo object
  //servoWheel.write(0);
  delay(2000);
}

void loop() {
  servoPull.write(180);
  delay(500);
  servoWheel.write(160);
  delay(750);
  servoWheel.write(20);
  delay(400);
  servoPull.write(0);
  servoWheel.write(90);
  delay(500);
  
}
