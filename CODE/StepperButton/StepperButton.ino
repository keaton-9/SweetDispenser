#include <AccelStepper.h>
#include <MultiStepper.h>

// Define step constants
#define FULLSTEP 4
#define FULLSTEP 8

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper GearMotor (FULLSTEP, 8, 10, 9, 11);
AccelStepper FeedMotor (FULLSTEP, 4, 6, 5, 7);

int RedIn = 22;    // pushbutton connected to digital pin 7
int GreenIn = 23;    // pushbutton connected to digital pin 7
int OrangeIn = 24;    // pushbutton connected to digital pin 7
int PurpleIn = 25;    // pushbutton connected to digital pin 7


void setup() {
  Serial.begin(9600);
  pinMode(RedIn, INPUT);    // sets the digital pin 7 as input
  pinMode(GreenIn, INPUT);    // sets the digital pin 7 as input
  pinMode(OrangeIn, INPUT);    // sets the digital pin 7 as input
  pinMode(PurpleIn, INPUT);    // sets the digital pin 7 as input

  /*
    // set the maximum speed, acceleration factor,
    // initial speed and the target position for motor 1
    GearMotor.setMaxSpeed(500.0);   //this is a good speed for the gear motor, need to refine how many steps i want it to take.
    GearMotor.setAcceleration(50.0); // the gear motor running on 6V D batteries is way better and capable
    GearMotor.setSpeed(200);
    GearMotor.moveTo(2038);
  */
  // set the same for motor 2
  FeedMotor.setMaxSpeed(500.0);
  FeedMotor.setAcceleration(200.0);
  FeedMotor.setSpeed(500);


}


void loop() {

  int inputs = ((digitalRead(RedIn) << 3) |  (digitalRead(GreenIn) << 2) | (digitalRead(OrangeIn) << 1) | digitalRead(PurpleIn));

  switch (inputs) {
    case 0: //no button
      Serial.println("NO INPUT");
      break;
    case 1: //purple
      Serial.println("PURPLE HIGH");
      FeedMotor.moveTo(FeedMotor.currentPosition() - 4076);
      FeedMotor.run();
      while (FeedMotor.distanceToGo() < 0) {
        FeedMotor.run();

      }
      break;
    case 2:
      Serial.println("ORANGE HIGH");
      break;
    case 4:
      Serial.println("GREEN HIGH");
      break;
    case 8:
      Serial.println("RED HIGH");
      break;
    default:
      Serial.println("NO INPUT");
      break;
  }
}
//if
// Change direction once the motor reaches target position

// if (GearMotor.distanceToGo() == 0)
// GearMotor.moveTo(-GearMotor.currentPosition());

//  if (FeedMotor.distanceToGo() == 0)
// FeedMotor.moveTo(-FeedMotor.currentPosition()-2038);
// FeedMotor.moveTo(-2038);
// Move the motor one step
// GearMotor.run();
//  FeedMotor.run();

//}
