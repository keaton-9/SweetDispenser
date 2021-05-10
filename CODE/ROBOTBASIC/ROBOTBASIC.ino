
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *M1 = AFMS.getMotor(1); //claw
Adafruit_DCMotor *M2 = AFMS.getMotor(2); //wrist
Adafruit_DCMotor *M3 = AFMS.getMotor(3); //elbow
Adafruit_DCMotor *M4 = AFMS.getMotor(4); //shoulder

void setup() {

  AFMS.begin();
  M1->setSpeed(150);
  M2->setSpeed(150);
  M3->setSpeed(150);
  M4->setSpeed(150);
}

void loop() {


//  M1->run(FORWARD); //claw CLOSE
//  delay (100);
//  M1->run(RELEASE); 
//  M1->run(BACKWARD); //CLAW OPEN
//  delay (100);
//  M1->run(RELEASE);

//  M2->run(FORWARD); //WRIST DOWN
//  delay (100);
//  M2->run(RELEASE);
  M2->run(BACKWARD); //WRIST UP
  delay (100);
  M2->run(RELEASE);

//  M3->run(FORWARD); //ELBOW UP (TOWARD BATTERIES)
//  delay (100);
//  M3->run(RELEASE);
//  M3->run(BACKWARD); //elbow DOWN (AWAY FROM BATTERIES)
//  delay (100);
//  M3->run(RELEASE);

  M4->run(FORWARD); //SHOULDER FORWARD
  delay (100);
  M4->run(RELEASE);
//  M4->run(BACKWARD); //SHOULDER BACKWARDS
//  delay (100);
//  M4->run(RELEASE);

  delay (5000);

}
