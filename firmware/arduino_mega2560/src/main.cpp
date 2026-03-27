/**
 * Test motor
 * 
 * This is a simple test sketch to verify that the ESCON motor driver is working 
 * correctly.
 */
#include <Arduino.h>
#include <robot_config.h>
#include <escon_driver.h>

EsconDriver leftMotor(PIN_LEFT_MOTOR_PWM, PIN_LEFT_MOTOR_EN, PIN_LEFT_MOTOR_DIR, 
                        PIN_LEFT_MOTOR_READY, PIN_LEFT_MOTOR_SPEED_ANA, PIN_LEFT_MOTOR_CURR_ANA);
// EsconDriver rightMotor(PIN_RIGHT_MOTOR_PWM, PIN_RIGHT_MOTOR_EN, PIN_RIGHT_MOTOR_DIR, 
//                         PIN_RIGHT_MOTOR_READY, PIN_RIGHT_MOTOR_SPEED_ANA, PIN_RIGHT_MOTOR_CURR_ANA);

char option;
int speed = 0;

void setup()
{
  //initialize the serial port
  Serial.begin(9600);

  // initialize Escon motor drivers
  leftMotor.init();
  // rightMotor.init();

  Serial.println("Waiting for 't' command to start motor test...");
}

void loop()
{
  if (Serial.available()>0){
    //read the sent option
    option=Serial.read();
    if(option=='t') {
      Serial.println("Beginning motor test...");
      leftMotor.setSpeed(speed);

      Serial.print("Set speed: ");
      Serial.println(speed);

      speed += -500; // Increase speed by 500 RPM every time 't' is sent
      if (speed > MOTOR_MAX_PERMISSIBLE_RPM) {
        speed = 0; // Reset speed to 0 after reaching max
      }
    } else {
      Serial.println("Unknown command. Please send 't' to start the motor test.");
    }
  }
}