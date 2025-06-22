// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DataLogUtil;

public class ElevatorCalibration extends Command {
  private Elevator elevator;
  
  private double percentOutput, rampRate;
  private final Timer timer = new Timer();

  private enum CalibrationRoutineState {
    RAMP_UP,
    STOP_UP,
    RAMP_DOWN,
    STOP_DOWN,
  }
  private CalibrationRoutineState state;

  /**
   * Ramps elevator speed upwards and then downwards, reversing when 10 inches from top or bottom.
   * The data collected during this routine can be used to calculate the terms of the elevator profile.
   * @param rampRate Ramp rate in percent output/second 
   * @param elevator Elevator subsystem
   */
  public ElevatorCalibration(double rampRate, Elevator elevator) {
    this.elevator = elevator;
    
    this.rampRate = rampRate;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    state = CalibrationRoutineState.RAMP_UP;

    elevator.enableFastLogging(true);
    DataLogUtil.writeMessage("ElevatorCalibration: Init, rampRate = ", rampRate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currTime = timer.get();

    if (!elevator.isElevatorCalibrated()) return;

    switch (state) {
      // Ramp upwards until 10 inches from top
      case RAMP_UP:
        if (elevator.getElevatorPosition() < ElevatorPosition.UPPER_LIMIT.value - 10.0) {
          percentOutput = MathUtil.clamp(currTime * rampRate, -1.0, 1.0);
          elevator.setElevatorPercentOutput(percentOutput);      
        } else {
          state = CalibrationRoutineState.STOP_UP;
          timer.reset();
          timer.start();
        }
        break;

      // Stop motor for 2 seconds
      case STOP_UP:
        if (currTime < 2.0) {
          elevator.stopElevatorMotors();     
        } else {
          state = CalibrationRoutineState.RAMP_DOWN;
          timer.reset();
          timer.start();
        }
        break;
      
      // Ramp downwards until 10 inches from bottom
      case RAMP_DOWN:
        if (elevator.getElevatorPosition() > ElevatorConstants.ElevatorPosition.LOWER_LIMIT.value + 10.0) {
          percentOutput = MathUtil.clamp(-currTime * rampRate, -1.0, 1.0);
          elevator.setElevatorPercentOutput(percentOutput);      
        } else {
          state = CalibrationRoutineState.STOP_DOWN;
          timer.reset();
          timer.start();
        }
        break;

      // Stop motor for 2 seconds
      case STOP_DOWN:
      default:
        if (currTime < 2.0) {
          elevator.stopElevatorMotors();     
        } else {
          state = CalibrationRoutineState.RAMP_UP;
          timer.reset();
          timer.start();
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevatorMotors();
    elevator.enableFastLogging(false);
    DataLogUtil.writeMessage("ElevatorCalibration: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
