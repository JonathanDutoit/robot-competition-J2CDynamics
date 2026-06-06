#include <AccelStepper.h>
#include <do2/robot_config.hpp>

// Creates an instance - Pick the version you want to use and un-comment it. That's the only required change.
//AccelStepper myStepper(AccelStepper::FULL4WIRE, AIn1, AIn2, BIn1, BIn2);  // works for TB6612 (Bipolar, constant voltage, H-Bridge motor driver)
//AccelStepper myStepper(AccelStepper::FULL4WIRE, In1, In3, In2, In4);    // works for ULN2003 (Unipolar motor driver)
AccelStepper myStepper(AccelStepper::DRIVER, PIN_STEPPER_STEP, PIN_STEPPER_DIR);           // works for a4988 (Bipolar, constant current, step/direction driver)

bool move_requested = false;

void setup() {
  // set the maximum speed, acceleration factor,
  // and the target position
  myStepper.setEnablePin(PIN_STEPPER_ENABLE); // Enable pin for a4988
  myStepper.setPinsInverted(false, false, true); // Invert the enable pin for tmc2208 (active LOW)
  myStepper.setMaxSpeed(500.0); // Based on required speed
  myStepper.setAcceleration(250.0); // Based on required accel
  myStepper.moveTo(200); // Move 1 turn (200 steps for 1.8 degree stepper) in one direction

  //initialize the serial port
	Serial.begin(9600);

  myStepper.enableOutputs(); // Enable the motor outputs
}

void loop() {
  // Change direction once the motor reaches target position
  /*
    if (myStepper.distanceToGo() == 0)   // this form also works - pick your favorite!
    myStepper.moveTo(-myStepper.currentPosition());

    // Move the motor one step
    myStepper.run();
  */
 if(Serial.available()){
    char val = Serial.read();
    if(val != -1){
      switch(val){
        case 'w'://Move
          move_requested = true;
          break;
        case 'z':
          Serial.println("Hello");
          break;
        case 'x':
          myStepper.disableOutputs();
          Serial.println("Motor disabled");
          move_requested = false;
          break;
        case 's':
          myStepper.enableOutputs();
          Serial.println("Motor enabled");
          move_requested = true;
          break;
        default:
          Serial.println("Invalid command");
          move_requested = false;
          myStepper.disableOutputs();
          break;
      }
    }
  }

  if (!myStepper.run() and move_requested) {   // run() returns true as long as the final position has not been reached and speed is not 0.
    myStepper.moveTo(-myStepper.currentPosition());
  }
}