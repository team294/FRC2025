// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DataLogUtil;

public class ElevatorSetPercent extends Command {
  private final Elevator elevator;
  
  private double percent = 0.0;
  private boolean fromShuffleboard;
  private boolean endImmediately;

  /**
   * Sets the percent output of the elevator and ends immediately.
   * @param percent -1.0 to 1.0 (positive = up, negative = down)
   * @param elevator Elevator subsystem
   */
  public ElevatorSetPercent(double percent, boolean endImmediately, Elevator elevator) {
    this.percent = percent;
    this.endImmediately = endImmediately;
    this.elevator = elevator;
    
    this.fromShuffleboard = false;
    addRequirements(elevator);
  }

  /**
   * Sets the percent output of the elevator from Shuffleboard and ends immediately.
   * @param elevator Elevator subsystem
   */
  public ElevatorSetPercent(boolean endImmediately, Elevator elevator) {
    this.endImmediately = endImmediately;
    this.elevator = elevator;
    
    this.fromShuffleboard = true;
    addRequirements(elevator);

    if (SmartDashboard.getNumber("Elevator Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Elevator Percent", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) percent = SmartDashboard.getNumber("Elevator Percent", 0.0);
    elevator.setElevatorPercentOutput(percent);

    DataLogUtil.writeMessage("ElevatorSetPercent: Init, Percent = ", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!endImmediately || interrupted) elevator.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endImmediately) return true;
    else return false;
  }
}
