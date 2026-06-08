#include <Arduino.h>
#include <do2/robot_config.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>
#include <common_config.hpp>
#include <do2/controllers/sweeper_controller.hpp>
#include <do2/data/do2_command.hpp>
#include <do2/data/do2_state.hpp>
#include <common/scheduler/periodic_task.hpp>

// Global instances
Do2Command cmd;
Do2State state;

DRI0018DriverChannel leftMotor(PIN_LEFT_SWEEPER_PWM, PIN_LEFT_SWEEPER_DIR, 
							   PIN_LEFT_SWEEPER_CURR_SENSE);
DRI0018DriverChannel rightMotor(PIN_RIGHT_SWEEPER_PWM, PIN_RIGHT_SWEEPER_DIR, 
								PIN_RIGHT_SWEEPER_CURR_SENSE);

SweeperController sweepersController(&leftMotor, &rightMotor, cmd, state);

PeriodicTask currentSenseTask(100); // Check current sense every 100 ms

void setup(void)
{
	//initialize the serial port
	Serial.begin(9600);

	// initialize DRI0018 motor driver channels
	leftMotor.init();
	rightMotor.init();

	// initialize the differential drive controller
	sweepersController.init();

	Serial.println("Sweeper test bench started");
}

void loop(void)
{
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1){
		
		switch(val){
			case 'c':
				cmd.mode = SweeperMode::Collect;
				break;
			case 'd':
				cmd.mode = SweeperMode::Dropoff;
				break;
			case 'i':
				cmd.mode = SweeperMode::Idle;
				break;
			case 'h':
				Serial.println("Hello");
				break;
		}

		Serial.print("Sweeper Mode: ");
		Serial.println(static_cast<int>(cmd.mode));

		sweepersController.update();
    } else {
		Serial.println("Error reading serial input");
		cmd.mode = SweeperMode::Idle;
		sweepersController.update();
	}
  }

  if (currentSenseTask.ready()) {
	// Check if either motor is in a fault state
	if (!leftMotor.isReady()) {
	  Serial.println("Left motor fault detected!");
	  cmd.mode = SweeperMode::Idle; // Stop both motors if left motor is faulty
	}
	if (!rightMotor.isReady()) {
	  Serial.println("Right motor fault detected!");
	  cmd.mode = SweeperMode::Idle; // Stop both motors if right motor is faulty
	}
  }
}
