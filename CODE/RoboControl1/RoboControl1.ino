#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// https://robotics.kawasaki.com/ja1/xyz/en/1804-03/

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *M1 = AFMS.getMotor(1); //Claw motor
Adafruit_DCMotor *M2 = AFMS.getMotor(2); //wrist motor
Adafruit_DCMotor *M3 = AFMS.getMotor(3); // elbow motor
Adafruit_DCMotor *M4 = AFMS.getMotor(4); //shoulder motor

int StartPin = 9;
int val = 0;  // variable to store the value read

bool RoboRun = false ;

void setup() {

  pinMode(StartPin, INPUT);
  Serial.begin(9600);

  AFMS.begin();
  M1->setSpeed(150);
  M2->setSpeed(150);
  M3->setSpeed(150);
  M4->setSpeed(150);
}

void loop() {
  Serial.println("running loop");

  val = digitalRead(StartPin);  // read the input pin
  Serial.println(val);
  Serial.print( " val");

  if (val == HIGH)
  {
    Serial.println("Val = high");
    delay (500);
    RoboRun = true;
  }
  else
  {
    RoboRun = false;
    Serial.println("Val = low ");
    delay (500);
  }

  if (RoboRun == true )
  {
    Serial.println("RoboRun");
    Robot();
    RoboRun = false;
    Serial.println("RoboDone");
  }

  else
  {
  }
}


void Robot() {
  // Grab

  M2->run(FORWARD);      //down //Wrist Motor Controls
  delay (1050);
  M2->run(RELEASE);
  delay (1000);

  M4->run(BACKWARD);      //back shoulder
  delay (600);
  M4->run(RELEASE);
  delay (1000);

  M3->run(BACKWARD);    //away from batteries elbow
  delay (1450);
  M3->run(RELEASE);
  delay (1000);

  M1->run(FORWARD);  //CLOSE     //Claw Motor Controls
  delay (750);
  M1->run(RELEASE);
  delay (1000);

  delay (2000);

  //DELIVER

  M3->run(FORWARD);       //Elbow Motor Controls
  delay (3000);            //towards batteries
  M3->run(RELEASE);
  delay (1000);

  M2->run(BACKWARD);      //up WRIST
  delay (1150);
  M2->run(RELEASE);
  delay (1000);

  M4->run(FORWARD);      //forward //shoulder Motor Controls
  delay (2800);
  M4->run(RELEASE);
  delay (1000);

  M1->run(BACKWARD);    //OPEN JAWS
  delay (750);
  M1->run(RELEASE);
  delay (1000);

  delay (2000);

  //RETURN

  M4->run(BACKWARD);      //back SHOULDER
  delay (2500);
  M4->run(RELEASE);
  delay (1000);

  M3->run(BACKWARD);    //away from batteries ELBOW
  delay (750);
  M3->run(RELEASE);
  delay (1000);
  RoboRun = false;
}
