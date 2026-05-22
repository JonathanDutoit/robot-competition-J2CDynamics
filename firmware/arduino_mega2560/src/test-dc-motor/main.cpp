#include <Arduino.h>
#include <common_config.hpp>
#include <common/controllers/differential_drive_controller.hpp>
#include <do2/do2_config.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>

DRI0018DriverChannel leftMotor(PIN_LEFT_SWEEPER_PWM, PIN_LEFT_SWEEPER_DIR, 
							   PIN_LEFT_SWEEPER_CURR_SENSE);
DRI0018DriverChannel rightMotor(PIN_RIGHT_SWEEPER_PWM, PIN_RIGHT_SWEEPER_DIR, 
								PIN_RIGHT_SWEEPER_CURR_SENSE);

DifferentialDriveController sweepersController(&leftMotor, &rightMotor);

void setup(void)
{
	//initialize the serial port
	Serial.begin(9600);

	// initialize DRI0018 motor driver channels
	leftMotor.init();
	rightMotor.init();
	}

void loop(void)
{
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1){
		static float leftSpeed = 0.0f;
		static float rightSpeed = 0.0f;
		
		switch(val){
			case 'w'://Move Forward
				leftSpeed += DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				rightSpeed += DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				break;
			case 's'://Move Backward
				leftSpeed -= DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				rightSpeed -= DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				break;
			case 'a'://Turn Left
				leftSpeed -= DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				rightSpeed += DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				break;
			case 'd'://Turn Right
				leftSpeed += DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				rightSpeed -= DC_MOTOR_MAX_VELOCITY_RAD_SEC/10.0f;
				break;
			case 'z':
				Serial.println("Hello");
				break;
			case 'x':
				leftSpeed = 0.0f;
				rightSpeed = 0.0f;
				break;
		}

		Serial.print("Left Speed (rad/s): ");
		Serial.print(leftSpeed);
		Serial.print(" | Right Speed (rad/s): ");
		Serial.println(rightSpeed);

		sweepersController.setVelocities(leftSpeed, rightSpeed);
    } else {
		Serial.println("Error reading serial input");
		sweepersController.setVelocities(0.0f, 0.0f); // Stop the motors on error
	}
  }
}
