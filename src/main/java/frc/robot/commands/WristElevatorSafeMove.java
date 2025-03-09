// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.ElevatorWristRegions;
import frc.robot.utilities.FileLog;

public class WristElevatorSafeMove extends Command {
  // Subsystem and object references
  private final Elevator elevator;
  private final Wrist wrist;
  private final FileLog log;
  // Input parameters
  private final ElevatorWristPosition destPosition;
  private final ElevatorWristRegions.RegionType type;

  private enum MoveState {
    MOVE_WRIST_INTO_THIS_REGION,
    MOVE_TO_NEXT_REGION,
    MOVE_TO_POSITION_IN_THIS_REGION
  }

  private MoveState curState;
  ElevatorWristRegions.Region curRegion, destRegion;
  private boolean done;

  /**
   * Moves the wrist and elevator in sequence, accounting for interlocks and regions.
   * @param position position to move the elevator and wrist to (use ElevatorWwristConstants.ElevatorWristPosition)
   * @param type  Region type = CORAL_ONLY or STANDARD
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristElevatorSafeMove(ElevatorWristPosition position, ElevatorWristRegions.RegionType type,
            Elevator elevator, Wrist wrist, FileLog log) {
    this.destPosition = position;
    this.type = type;
    this.elevator = elevator;
    this.wrist = wrist;
    this.log = log;

    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!elevator.isElevatorCalibrated() || !wrist.isWristCalibrated()) {
      done = true;
      log.writeLog(false, "WristElevatorSafeMove", "Init", "Elev Calibrated", elevator.isElevatorCalibrated(),
          "Wrist calibrated", wrist.isWristCalibrated(), "Position", destPosition.toString(), "Type", type.toString());
      return;
    }

    log.writeLog(false, "WristElevatorSafeMove", "Init", "Calibrated", true, "Position", destPosition.toString(), "Type", type.toString());

    curRegion = ElevatorWristRegions.GetRegion(type, elevator.getElevatorPosition());
    destRegion = ElevatorWristRegions.GetRegion(type, destPosition.elevatorPosition);

    double curWristAngle = wrist.getWristAngle();
    if (curWristAngle <= curRegion.wristMin || curWristAngle >= curRegion.wristMax) {
      curState = MoveState.MOVE_WRIST_INTO_THIS_REGION;
    } else if (curRegion == destRegion) {
      curState = MoveState.MOVE_TO_POSITION_IN_THIS_REGION;
    } else {
      curState = MoveState.MOVE_TO_NEXT_REGION;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (done) return;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
