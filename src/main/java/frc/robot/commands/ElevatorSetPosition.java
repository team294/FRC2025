// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DataLogUtil;

public class ElevatorSetPosition extends Command {
  private final Elevator elevator;
  private final DataLogUtil log;
  private double target;
  private double tolerance = 0.5;
  private int toleranceCount = 0;

  /**
   * Sets the target position of the elevator to run a generated profile.
   * The command will end when the elevator is within 0.5 inches of the target position for 5 cycles.
   * @param target target position, in inches (0 = lower limit, positive = up)
   * @param elevator Elevator subsystem
   * @param log FileLog utility
   */
  public ElevatorSetPosition(double target, Elevator elevator, DataLogUtil log) {
    this.elevator = elevator;
    this.log = log;
    this.target = target;
    addRequirements(elevator);
  }

  /**
   * Sets the target position of the elevator to run a generated profile.
   * The command will end when the elevator is within 0.5 inches of the target position for 5 cycles.
   * @param target target ElevatorWristPosition, in inches (see ElevatorWristConstants.ElevatorWristPosition)
   * @param elevator Elevator subsystem
   * @param log FileLog utility
   */
  public ElevatorSetPosition(ElevatorWristPosition target, Elevator elevator, DataLogUtil log) {
    this.elevator = elevator;
    this.log = log;
    this.target = target.elevatorPosition;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO add interlocks with wrist, algaeGrabber, and coralEffector
    elevator.setElevatorProfileTarget(target);
    toleranceCount = 0;

    log.writeLog(false, "ElevatorSetPosition", "Init", "Target", target, "Position", elevator.getElevatorPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "ElevatorSetPosition", "End", "Target", target, "Position", elevator.getElevatorPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO add interlocks with wrist, algaeGrabber, and coralEffector
    if (!elevator.isElevatorCalibrated() || Math.abs(elevator.getElevatorPosition() - target) <= tolerance) {
      toleranceCount++;
      log.writeLog(false, "ElevatorSetPosition", "Within Tolerance", "Target", target, "Position", elevator.getElevatorPosition(), "Tolerance Count", toleranceCount);
    }
    return (toleranceCount > 5);
  }
}
