// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
    MOVE_WRIST_INTO_THIS_REGION_START,
    MOVE_WRIST_INTO_THIS_REGION_CONT,
    MOVE_TO_NEXT_REGION_START,
    MOVE_TO_NEXT_REGION_CONT,
    MOVE_TO_POSITION_IN_THIS_REGION_START,
    MOVE_TO_POSITION_IN_THIS_REGION_CONT,
    DONE
  }

  private MoveState curState;
  private ElevatorWristRegions.Region curRegion, nextRegion, destRegion;
  private boolean movingDown;     // True if elevator is moving down, false if moving up
  private final double wristBuffer = 3.0;          // How much to target to keep the wrist within min/max for each region, in degrees
  private final double elevatorBuffer = 2.0;       // How much to target to keep the elevator within min/max for each region, in inches

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
      curState = MoveState.DONE;
      log.writeLog(false, "WristElevatorSafeMove", "Init", "Elev Calibrated", elevator.isElevatorCalibrated(),
          "Wrist calibrated", wrist.isWristCalibrated(), "Position", destPosition.toString(), "Type", type.toString());
      return;
    }

    log.writeLog(false, "WristElevatorSafeMove", "Init", "Calibrated", true, "Position", destPosition.toString(), "Type", type.toString());

    curRegion = ElevatorWristRegions.GetRegion(type, elevator.getElevatorPosition());
    destRegion = ElevatorWristRegions.GetRegion(type, destPosition.elevatorPosition);

    double curWristAngle = wrist.getWristAngle();
    if (curWristAngle < curRegion.wristMin || curWristAngle > curRegion.wristMax) {
      curState = MoveState.MOVE_WRIST_INTO_THIS_REGION_START;
    } else if (curRegion == destRegion) {
      curState = MoveState.MOVE_TO_POSITION_IN_THIS_REGION_START;
    } else {
      curState = MoveState.MOVE_TO_NEXT_REGION_START;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO add some logging...

    switch (curState) {
      case DONE:
        return;
    
      case MOVE_WRIST_INTO_THIS_REGION_START:
        // State description:  Start moving wrist into the current region allowable range

        // Hold elevator from moving
        elevator.setElevatorProfileTarget(elevator.getElevatorPosition());
        // Start moving the wrist
        wrist.setWristAngle( MathUtil.clamp(wrist.getWristAngle(), curRegion.wristMin + wristBuffer, curRegion.wristMax - wristBuffer));

        // Advance state to wait for wrist to get into allowable range
        curState = MoveState.MOVE_WRIST_INTO_THIS_REGION_CONT;
        break;

      case MOVE_WRIST_INTO_THIS_REGION_CONT:
        // State description:  Waiting while moving wrist into the current region allowable range

        double curWristAngle = wrist.getWristAngle();
        if (curWristAngle >= curRegion.wristMin && curWristAngle <= curRegion.wristMax) {
          // The wrist is now in the allowed range for this region.
          // Advance state to move the elevator and wrist.
          if (curRegion == destRegion) {
            curState = MoveState.MOVE_TO_POSITION_IN_THIS_REGION_START;
          } else {
            curState = MoveState.MOVE_TO_NEXT_REGION_START;        }
          }
        break;

      case MOVE_TO_NEXT_REGION_START:
        // State description:  Start moving elevator and wrist to the edge of the next region

        // Determine which way to move
        movingDown = (curRegion.regionIndex > destRegion.regionIndex);
        if (movingDown) {
          // We are moving down

          // Get next region
          var optNextRegion = curRegion.getRegionBelow();
          if (optNextRegion.isEmpty()) {
            log.writeLog(false, "WristElevatorSafeMove", "Execute MOVE_TO_NEXT_REGION_START", "Empty Region Below", curRegion.toString());
            curState = MoveState.DONE;
            return;
          }
          nextRegion = optNextRegion.get();

          // Start moving the elevator
          elevator.setElevatorProfileTarget(curRegion.elevatorMin + elevatorBuffer);
        } else {
          // We are moving up

          // Get next region
          var optNextRegion = curRegion.getRegionAbove();
          if (optNextRegion.isEmpty()) {
            log.writeLog(false, "WristElevatorSafeMove", "Execute MOVE_TO_NEXT_REGION_START", "Empty Region Above", curRegion.toString());
            curState = MoveState.DONE;
            return;
          }
          nextRegion = optNextRegion.get();

          // Start moving the elevator
          elevator.setElevatorProfileTarget(curRegion.elevatorMax - elevatorBuffer);
        }

        // Start moving the wrist
        wrist.setWristAngle( MathUtil.clamp( 
          MathUtil.clamp(wrist.getWristAngle(), curRegion.wristMin + wristBuffer, curRegion.wristMax - wristBuffer),
          nextRegion.wristMin + wristBuffer, nextRegion.wristMax - wristBuffer)
        );
      
        // Advance state to wait for wrist to get into allowable range
        curState = MoveState.MOVE_TO_NEXT_REGION_CONT;
        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return curState == MoveState.DONE;
  }
}
