/**
 * Test motor
 * 
 * This is a simple test sketch to verify that the ESCON motor driver is working 
 * correctly.
 */
#include <Arduino.h>
#include <common/robot_config.hpp>
#include <common/drivers/escon_driver.hpp>

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
      Serial.println("Increase speed: 'u'");
      Serial.println("Decrease speed: 'd'");
      Serial.println("Stop motor: 's'");
      Serial.println("Get motor status: 'm'");
    } else if (option == 'u') {
      speed += 20; // Increase speed by 20 RPM every time 'u' is sent
    } else if (option == 'd') {
      speed -= 20; // Decrease speed by 20 RPM every time 'd' is sent
    } else if (option == 's') {
      speed = 0; // Stop the motor when 's' is sent
    } else if (option == 'm') {
      Serial.print("Current speed: ");
      Serial.print(leftMotor.getSpeed());
      Serial.print(" RPM, Current: ");
      Serial.print(leftMotor.getCurrent());
      Serial.println(" mA");
    } else {
      Serial.println("Unknown command. Please send 't' to start the motor test.");
    }

    leftMotor.setSpeed(speed);

    Serial.print("Set speed: ");
    Serial.println(speed);
  }
}