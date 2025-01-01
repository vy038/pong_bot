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

#include <AFMotor.h>
#include <Servo.h>
#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// DC motors
AF_DCMotor motorRB(1);
AF_DCMotor motorRF(2);
AF_DCMotor motorLF(3);
AF_DCMotor motorLB(4);

Servo servoPull;
Servo servoWheel;

int leftIRSensor = 15;
int backIRSensor = 16;
int rightIRSensor = 17;

unsigned long startMillisFiring;  //some global variables available anywhere in the program
unsigned long currentMillisFiring;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;

int decision;
int fireOrNo = 1;


const int inSpeedRB = 100;
const int inSpeedRF = 100;
const int inSpeedLF = 150;
const int inSpeedLB = 150;
int speedRB = inSpeedRB;
int speedRF = inSpeedRF;
int speedLF = inSpeedLF;
int speedLB = inSpeedLB;

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps

  pinMode(leftIRSensor, INPUT);
  pinMode(backIRSensor, INPUT);
  pinMode(rightIRSensor, INPUT);


  servoPull.attach(9);
  servoPull.write(180);

  motorRB.setSpeed(speedRB);
  motorRF.setSpeed(speedRF);
  motorLF.setSpeed(speedLF);
  motorLB.setSpeed(speedLB);
  motorRB.run(RELEASE);
  motorRF.run(RELEASE);
  motorLF.run(RELEASE);
  motorLB.run(RELEASE);

  randomSeed(analogRead(0));

  servoWheel.attach(10);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  delay(20);
  
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

      speedRF = inSpeedRF + pitch*2.5;
      speedLF = inSpeedLF - pitch*2.5;
    } else if (decision == 2) {
      forward(1);
      Serial.println("bck");

      speedRF = inSpeedRF - pitch*2.5;
      speedLF = inSpeedLF + pitch*2.5;
    } else if (decision == 3) {
      side(1);
      Serial.println("left");

      speedRF = inSpeedRF + pitch*2.5;
      speedLF = inSpeedLF + pitch*2.5;
    } else if (decision == 4) {
      side(0);
      Serial.println("right");

      speedRF = inSpeedRF - pitch*2.5;
      speedLF = inSpeedLF - pitch*2.5;
    }

    
    motorRB.setSpeed(speedRB);
    motorRF.setSpeed(speedRF);
    motorLF.setSpeed(speedLF);
    motorLB.setSpeed(speedLB);

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

  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  gyroAngleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  gyroAngleY = 0.96 * gyroAngleY + 0.04 * accAngleY;

  roll = gyroAngleX;
  pitch = gyroAngleY;

  Serial.println(pitch);
  Serial.print("RB: ");
  Serial.print(speedRB);
  Serial.print(" RF: ");
  Serial.print(speedRF);
  Serial.print(" LF: ");
  Serial.print(speedLF);
  Serial.print(" LB: ");
  Serial.println(speedLB);

  


  //delay(5); //save resources
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

void side(int dir) {
  switch (dir) {
    case 0:  //to the right
      motorRB.run(BACKWARD);
      motorRF.run(FORWARD);
      motorLF.run(BACKWARD);
      motorLB.run(FORWARD);
      break;
    case 1:  //to the left
      motorRB.run(FORWARD);
      motorRF.run(BACKWARD);
      motorLF.run(FORWARD);
      motorLB.run(BACKWARD);
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

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}