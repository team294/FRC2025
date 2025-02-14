// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.utilities.FileLog;

public class ElevatorSetPosition extends Command {
  private final FileLog log;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private double target;
  private int toleranceCount = 0;

  public ElevatorSetPosition(double target, Elevator elevator, EndEffector endEffector, FileLog log) {
    this.log = log;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.target = target;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.checkAndZeroElevatorEncoders();
    if (!endEffector.isPiecePresentInEntry()) elevator.setElevatorProfileTarget(target);
    toleranceCount = 0;
    log.writeLog(false, "ElevatorSetPosition", "Initialize", "Target", target, "Position", elevator.getElevatorPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "ElevatorSetPosition", "End", "Target", target, "Position", elevator.getElevatorPosition());
    elevator.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!elevator.isElevatorCalibrated() || endEffector.isPiecePresentInEntry() || Math.abs(elevator.getElevatorPosition() - target) <= 0.5) {
      toleranceCount++;
      log.writeLog(false, "ElevatorSetPosition", "Within Tolerance", "Target", target, "Position", elevator.getElevatorPosition(), "Tolerance Count", toleranceCount);
    }
    return (toleranceCount > 5);
  }
}
