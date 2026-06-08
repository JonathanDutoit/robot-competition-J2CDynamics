/**
 * TODO: Add description
 */
#include <Arduino.h>
#include <do2/robot_config.hpp>
#include <do2/drivers/dri0018_driver_channel.hpp>
#include <do2/communication/sweeper_command_handler.hpp>
#include <do2/data/do2_command.hpp>
#include <do2/data/do2_state.hpp>
#include <common_config.hpp>
#include <common/drivers/escon_driver.hpp>
#include <common/controllers/differential_drive_controller.hpp>
#include <common/sensors/duplo_counter.hpp>
#include <common/scheduler/periodic_task.hpp>
#include <common/communication/serial_bridge.hpp>
#include <common/data/robot_command.hpp>
#include <common/data/robot_state.hpp>

// Global instances
Do2Command cmd;
Do2State state;

SerialBridge serialBridge(cmd, state);
SweeperCommandHandler sweeperHandler(cmd);

EsconDriver leftMotor(PIN_LEFT_MAXON_PWM, PIN_LEFT_MAXON_EN, PIN_LEFT_MAXON_DIR, 
                        PIN_LEFT_MAXON_READY, PIN_LEFT_MAXON_SPEED_ANA, 
                        PIN_LEFT_MAXON_CURR_ANA);
EsconDriver rightMotor(PIN_RIGHT_MAXON_PWM, PIN_RIGHT_MAXON_EN, PIN_RIGHT_MAXON_DIR, 
                        PIN_RIGHT_MAXON_READY, PIN_RIGHT_MAXON_SPEED_ANA, 
                        PIN_RIGHT_MAXON_CURR_ANA);
DifferentialDriveController driveController(&leftMotor, &rightMotor, cmd, state); // Example wheel diameter and gear ratio

DRI0018DriverChannel leftSweeper(PIN_LEFT_SWEEPER_PWM, PIN_LEFT_SWEEPER_DIR, 
								PIN_LEFT_SWEEPER_CURR_SENSE);
DRI0018DriverChannel rightSweeper(PIN_RIGHT_SWEEPER_PWM, PIN_RIGHT_SWEEPER_DIR, 
								 PIN_RIGHT_SWEEPER_CURR_SENSE);
DifferentialDriveController sweepersController(&leftSweeper, &rightSweeper, cmd, state);

DuploCounter duploCounter(PIN_DUPLO_IR_SENSOR);

// Define tasks schedule
PeriodicTask controlTask(20); // 20 ms period for control loop (50 Hz)
PeriodicTask duploTask(100); // 100 ms period for duplo counter update (10 Hz)
PeriodicTask sweepersTask(20); // 50 ms period for sweepers control loop (50 Hz)

void setup()
{
  // Initialize components
  leftMotor.init();
  rightMotor.init();
  driveController.init();

  leftSweeper.init();
  rightSweeper.init();
  sweepersController.init();

  duploCounter.init();

  serialBridge.init();
  if (!serialBridge.registerHandler(&sweeperHandler)) {
    Serial.println("ERROR: Failed to register SWEEPER command handler");
  }

  Serial.println("Robot initialized");
}

void loop()
{
  if (controlTask.ready()){
      driveController.update();
  }

  if (duploTask.ready()){
      duploCounter.update();
      state.duploCount = duploCounter.getCount();
  }

  if (sweepersTask.ready()){
      sweepersController.update();
  }

  serialBridge.update();
}