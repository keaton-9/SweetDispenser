#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *M1 = AFMS.getMotor(1); //naming motor1-4
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);

void setup(){
  
  AFMS.begin();
  M1->setSpeed(150);
    M2->setSpeed(150);
      M3->setSpeed(150);
        M4->setSpeed(150);
}

void loop(){
  M4->run(FORWARD);
  delay (500);
  M4->run(RELEASE);
  delay (50);
  M4->run(BACKWARD);
  delay (500);
  M4->run(RELEASE);
}
