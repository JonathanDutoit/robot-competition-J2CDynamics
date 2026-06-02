#include <AccelStepper.h>

const int dirPin = 39;
const int stepPin = 41;

// Creates an instance - Pick the version you want to use and un-comment it. That's the only required change.
//AccelStepper myStepper(AccelStepper::FULL4WIRE, AIn1, AIn2, BIn1, BIn2);  // works for TB6612 (Bipolar, constant voltage, H-Bridge motor driver)
//AccelStepper myStepper(AccelStepper::FULL4WIRE, In1, In3, In2, In4);    // works for ULN2003 (Unipolar motor driver)
AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);           // works for a4988 (Bipolar, constant current, step/direction driver)

void setup() {
  // set the maximum speed, acceleration factor,
  // and the target position
  myStepper.setMaxSpeed(500.0); // Based on required speed
  myStepper.setAcceleration(250.0); // Based on required accel
  myStepper.moveTo(200); // Move 1 turn (200 steps for 1.8 degree stepper) in one direction
}

void loop() {
  // Change direction once the motor reaches target position
  /*
    if (myStepper.distanceToGo() == 0)   // this form also works - pick your favorite!
    myStepper.moveTo(-myStepper.currentPosition());

    // Move the motor one step
    myStepper.run();
  */
  if (!myStepper.run()) {   // run() returns true as long as the final position has not been reached and speed is not 0.
    myStepper.moveTo(-myStepper.currentPosition());
  }
}