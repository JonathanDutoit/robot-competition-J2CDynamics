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

// Global instances
RobotCommand cmd;
RobotState robotState;

SerialBridge serialBridge(cmd, robotState);

EsconDriver leftMotor(PIN_LEFT_MAXON_PWM, PIN_LEFT_MAXON_EN, PIN_LEFT_MAXON_DIR, 
                        PIN_LEFT_MAXON_READY, PIN_LEFT_MAXON_SPEED_ANA, 
                        PIN_LEFT_MAXON_CURR_ANA);
EsconDriver rightMotor(PIN_RIGHT_MAXON_PWM, PIN_RIGHT_MAXON_EN, PIN_RIGHT_MAXON_DIR, 
                        PIN_RIGHT_MAXON_READY, PIN_RIGHT_MAXON_SPEED_ANA, 
                        PIN_RIGHT_MAXON_CURR_ANA);
DifferentialDriveController driveController(&leftMotor, &rightMotor, cmd, robotState);

DuploCounter duploCounter(PIN_DUPLO_IR_SENSOR);

// Define tasks schedule
PeriodicTask controlTask(20); // 20 ms period for control loop (50 Hz)
PeriodicTask duploTask(100); // 100 ms period for duplo counter update (10 Hz)

void setup()
{
  // Initialize components
  leftMotor.init();
  rightMotor.init();

  // Offset handling
  delay(100);  // let ADC settle, motors disabled
  leftMotor.setZeroVoltage(leftMotor.measureZeroVoltage(250));
  rightMotor.setZeroVoltage(rightMotor.measureZeroVoltage(250));
  
  driveController.init();
  duploCounter.init();
  serialBridge.init();

  Serial.println("Robot initialized");

  // Offset handling
  delay(100);  // let ADC settle, motors disabled
  leftMotor.setZeroVoltage(leftMotor.measureZeroVoltage(250));
  rightMotor.setZeroVoltage(rightMotor.measureZeroVoltage(250));
}

void loop()
{
  if (controlTask.ready()){
      driveController.update();
  }

  if (duploTask.ready()){
      duploCounter.update();
      robotState.duploCount = duploCounter.getCount();
  }

  serialBridge.update();
}