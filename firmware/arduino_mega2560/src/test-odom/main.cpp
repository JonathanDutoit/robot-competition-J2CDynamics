/**
 * TODO: Add description
 */
#include <Arduino.h>
#include <common_config.hpp>
#include <common/drivers/escon_driver.hpp>
#include <common/controllers/differential_drive_controller.hpp>
#include <common/sensors/duplo_counter.hpp>
#include <common/scheduler/periodic_task.hpp>
#include <common/communication/serial_bridge.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>

#define MIN_VEL_STEP_RAD_SEC 0.035f

// Global instances
RobotCommand cmd;
RobotState robotState;

EsconDriver leftMotor(PIN_LEFT_MAXON_PWM, PIN_LEFT_MAXON_EN, PIN_LEFT_MAXON_DIR, 
                        PIN_LEFT_MAXON_READY, PIN_LEFT_MAXON_SPEED_ANA, 
                        PIN_LEFT_MAXON_CURR_ANA);
EsconDriver rightMotor(PIN_RIGHT_MAXON_PWM, PIN_RIGHT_MAXON_EN, PIN_RIGHT_MAXON_DIR, 
                        PIN_RIGHT_MAXON_READY, PIN_RIGHT_MAXON_SPEED_ANA, 
                        PIN_RIGHT_MAXON_CURR_ANA);
DifferentialDriveController driveController(&leftMotor, &rightMotor, cmd, robotState);

// Define tasks schedule
PeriodicTask controlTask(20); // 20 ms period for control loop (50 Hz)

void setup()
{
  // Initialize components
  leftMotor.init();
  rightMotor.init();
  driveController.init();

  Serial.begin(9600);

  Serial.println("Odom test bench started");
}

void loop()
{
    if(Serial.available()){
        char val = Serial.read();
        if(val != -1){
            static float leftSpeed = 0.0f;
            static float rightSpeed = 0.0f;
            
            switch(val){
                case 'w'://Move Forward
                    leftSpeed += MIN_VEL_STEP_RAD_SEC;
                    rightSpeed += MIN_VEL_STEP_RAD_SEC;
                    break;
                case 's'://Move Backward
                    leftSpeed -= MIN_VEL_STEP_RAD_SEC;
                    rightSpeed -= MIN_VEL_STEP_RAD_SEC;
                    break;
                case 'a'://Turn Left
                    leftSpeed -= MIN_VEL_STEP_RAD_SEC;
                    rightSpeed += MIN_VEL_STEP_RAD_SEC;
                    break;
                case 'd'://Turn Right
                    leftSpeed += MIN_VEL_STEP_RAD_SEC;
                    rightSpeed -= MIN_VEL_STEP_RAD_SEC;
                    break;
                case 'z':
                    Serial.println("Hello");
                    break;
                case 'x':
                    leftSpeed = 0.0f;
                    rightSpeed = 0.0f;
                    break;
                case 'm': 
                    Serial.print("State left wheel velocity: ");
                    Serial.print(robotState.leftWheelVelocity);
                    Serial.print("State right wheel velocity: ");
                    Serial.print(robotState.rightWheelVelocity);
                    break;
            }

            Serial.print("Left Speed (rad/s): ");
            Serial.print(leftSpeed);
            Serial.print(" | Right Speed (rad/s): ");
            Serial.println(rightSpeed);

            cmd.leftWheelVelocity = leftSpeed;
            cmd.rightWheelVelocity = rightSpeed;

            driveController.update();
        } else {
            Serial.println("Error reading serial input");
            cmd.leftWheelVelocity = 0.0f;
            cmd.rightWheelVelocity = 0.0f;
        }

        if (controlTask.ready()){
            driveController.update();
        }
    }
}