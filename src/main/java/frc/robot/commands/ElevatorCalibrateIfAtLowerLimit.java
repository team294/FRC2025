// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

public class ElevatorCalibrateIfAtLowerLimit extends Command {
  private final Elevator elevator;
  private final FileLog log;

  /**
   * If the elevator is at the lower limit, calibrate the encoders.
   * @param elevator Elevator subsystem
   * @param log FileLog log
   */
  public ElevatorCalibrateIfAtLowerLimit(Elevator elevator, FileLog log) {
    this.log = log;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.checkAndZeroElevatorEncoders();
    log.writeLog(false, "ElevatorCalibrateIfAtLowerLimit", "Initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
