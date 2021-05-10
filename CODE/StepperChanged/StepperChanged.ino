#include <AccelStepper.h>
#include <MultiStepper.h>

// Define step constants
#define FULLSTEP 4
#define FULLSTEP 8

// Creates two instances
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper GearMotor (FULLSTEP, 8, 10, 9, 11);
AccelStepper FeedMotor (FULLSTEP, 4, 6, 5, 7);

void setup() {

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
  FeedMotor.setAcceleration(50.0);
  FeedMotor.setSpeed(200);
  FeedMotor.moveTo(-2038);
  
}

void loop() {
  // Change direction once the motor reaches target position
  
 // if (GearMotor.distanceToGo() == 0) 
   // GearMotor.moveTo(-GearMotor.currentPosition());
    
  if (FeedMotor.distanceToGo() == 0) 
    FeedMotor.moveTo(-FeedMotor.currentPosition());

  // Move the motor one step
 // GearMotor.run();
 FeedMotor.run();
}
