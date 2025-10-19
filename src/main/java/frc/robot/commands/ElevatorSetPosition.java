// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DataLogUtil;

public class ElevatorSetPosition extends Command {
  private final Elevator elevator;
  
  private double target;
  private double tolerance = 0.5;
  private int toleranceCount = 0;
  private boolean fromShuffleboard;

  /**
   * Sets the target position of the elevator to run a generated profile.
   * The command will end when the elevator is within 0.5 inches of the target position for 5 cycles.
   * @param target target position, in inches (0 = lower limit, positive = up)
   * @param elevator Elevator subsystem
   */
  public ElevatorSetPosition(double target, Elevator elevator) {
    this.elevator = elevator;
    this.target = target;
    fromShuffleboard = false;
    addRequirements(elevator);
  }

  /**
   * Sets the target position of the elevator to run a generated profile.
   * The command will end when the elevator is within 0.5 inches of the target position for 5 cycles.
   * @param target target from ElevatorWristPosition enum, in inches (see ElevatorWristConstants.ElevatorWristPosition)
   * @param elevator Elevator subsystem
   */
  public ElevatorSetPosition(ElevatorWristPosition target, Elevator elevator) {
    this.elevator = elevator;
    this.target = target.elevatorPosition;
    fromShuffleboard = false;
    addRequirements(elevator);
  }

  /**
   * Sets the target position of the elevator to run a generated profile.  Gets the target position from Suffleboard.
   * The command will end when the elevator is within 0.5 inches of the target position for 5 cycles.
   * @param elevator Elevator subsystem
   */
  public ElevatorSetPosition(Elevator elevator) {
    this.elevator = elevator;
    fromShuffleboard = true;
    addRequirements(elevator);

    if (SmartDashboard.getNumber("Elevator Goal Position", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Elevator Goal Position", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO add interlocks with wrist, algaeGrabber, and coralEffector
    if (fromShuffleboard) target = SmartDashboard.getNumber("Elevator Goal Position", 0.0);

    elevator.setElevatorProfileTarget(target);
    toleranceCount = 0;

    DataLogUtil.writeMessage("ElevatorSetPosition: Init, Target =", target, ", Position =", elevator.getElevatorPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("ElevatorSetPosition: End, Target =", target, ", Position =", elevator.getElevatorPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO add interlocks with wrist, algaeGrabber, and coralEffector
    if (!elevator.isElevatorCalibrated() || Math.abs(elevator.getElevatorPosition() - target) <= tolerance) {
      toleranceCount++;
      DataLogUtil.writeMessage("ElevatorSetPosition: Within Tolerance, Target =", target, ", Position =", elevator.getElevatorPosition(), ", Tolerance Count =", toleranceCount);
    }
    return (toleranceCount > 5);
  }
}
