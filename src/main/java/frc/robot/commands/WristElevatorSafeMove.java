// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Currency;

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
    MOVE_TO_NEXT_REGION_CONT_WRIST_ELEV,
    MOVE_TO_NEXT_REGION_CONT_ELEV_ONLY,
    MOVE_TO_POSITION_IN_THIS_REGION_START,
    MOVE_TO_POSITION_IN_THIS_REGION_CONT,
    DONE,
    DONE_ERROR
  }

  private MoveState curState;
  private ElevatorWristRegions.Region curRegion, nextRegion, destRegion;
  private boolean movingDown;     // True if elevator is moving down, false if moving up
  private final double wristBuffer = 3.0;       // How much to target to keep the wrist within min/max for each region, in degrees
  private final double wristTargetTol = 3.0;    // Wrist tolerance to goal postion for this command to end, in degrees
  private final double elevatorMovingTol = 2.0; // Elevator tolerances when moving between regions, in inches
  private final double elevatorTargetTol = 1.0; // Elevator tolerance to goal postion for this command to end, in inches

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
      curState = MoveState.DONE_ERROR;
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
    double curWristAngle = wrist.getWristAngle();
    double curElevPos = elevator.getElevatorPosition();

    // TODO add some logging...

    switch (curState) {
      case DONE:
      case DONE_ERROR:
        return;
    
      case MOVE_WRIST_INTO_THIS_REGION_START:
        // State description:  Start moving wrist into the current region allowable range (don't move the elevator)

        // Hold elevator from moving
        elevator.setElevatorProfileTarget(curElevPos);
        // Start moving the wrist
        wrist.setWristAngle( MathUtil.clamp(curWristAngle, curRegion.wristMin + wristBuffer, curRegion.wristMax - wristBuffer));

        // Advance state to wait for wrist to get into allowable range
        curState = MoveState.MOVE_WRIST_INTO_THIS_REGION_CONT;
        break;

      case MOVE_WRIST_INTO_THIS_REGION_CONT:
        // State description:  Waiting while moving wrist into the current region allowable range

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
            curState = MoveState.DONE_ERROR;
            return;
          }
          nextRegion = optNextRegion.get();

          // Start moving the elevator
          elevator.setElevatorProfileTarget(curRegion.elevatorMin);
        } else {
          // We are moving up

          // Get next region
          var optNextRegion = curRegion.getRegionAbove();
          if (optNextRegion.isEmpty()) {
            log.writeLog(false, "WristElevatorSafeMove", "Execute MOVE_TO_NEXT_REGION_START", "Empty Region Above", curRegion.toString());
            curState = MoveState.DONE_ERROR;
            return;
          }
          nextRegion = optNextRegion.get();

          // Start moving the elevator
          elevator.setElevatorProfileTarget(curRegion.elevatorMax);
        }

        // Start moving the wrist
        // Choose an angle that is safe for both the current region and the next region
        wrist.setWristAngle( MathUtil.clamp( 
          MathUtil.clamp(curWristAngle, curRegion.wristMin + wristBuffer, curRegion.wristMax - wristBuffer),
          nextRegion.wristMin + wristBuffer, nextRegion.wristMax - wristBuffer)
        );
      
        // Advance state to wait for wrist to get into allowable range
        curState = MoveState.MOVE_TO_NEXT_REGION_CONT_WRIST_ELEV;
        break;

      case MOVE_TO_NEXT_REGION_CONT_WRIST_ELEV:
        // State description:  Continue moving elevator and wrist to the edge of the next region

        // Wait in this state (don't do anything) if wrist is not safe to go into next region

        if (curWristAngle >= curRegion.wristMin && curWristAngle <= curRegion.wristMax && 
            curWristAngle >= nextRegion.wristMin && curWristAngle <= nextRegion.wristMax) {
          // Wrist is now safe to go into next region
          
          if ( (movingDown && curElevPos <= curRegion.elevatorMin + elevatorMovingTol) ||
               (!movingDown && curElevPos >= curRegion.elevatorMax - elevatorMovingTol ) ) {
            // Elevator is ready to go into next region

            // Advance to next region
            curRegion = nextRegion;

            // Advance to next state
            if (curRegion == destRegion) {
              curState = MoveState.MOVE_TO_POSITION_IN_THIS_REGION_START;
            } else {
              curState = MoveState.MOVE_TO_NEXT_REGION_START;
            }
          } else {
            // Elevator has further to go in this region

            // Change elevator target for continuous motion into next region
            if (nextRegion == destRegion) {
              elevator.setElevatorProfileTarget( destPosition.elevatorPosition );
            } else {
              elevator.setElevatorProfileTarget( (movingDown) ? nextRegion.elevatorMin : nextRegion.elevatorMax );
            }

            // Advance to next state
            curState = MoveState.MOVE_TO_NEXT_REGION_CONT_ELEV_ONLY;
          }
        }
        break;

      case MOVE_TO_NEXT_REGION_CONT_ELEV_ONLY:
        // State description:  Continue moving elevator to the edge of the next region.  Wrist is ready for next region

        // Wait in this state until the elevator is at the next region
          
        if ( (movingDown && curElevPos <= curRegion.elevatorMin + elevatorMovingTol) ||
              (!movingDown && curElevPos >= curRegion.elevatorMax - elevatorMovingTol ) ) {
          // Elevator is ready to go into next region

          // Advance to next region
          curRegion = nextRegion;

          // Advance to next state
          if (curRegion == destRegion) {
            curState = MoveState.MOVE_TO_POSITION_IN_THIS_REGION_START;
          } else {
            curState = MoveState.MOVE_TO_NEXT_REGION_START;
          }
        } 
        break;

      case MOVE_TO_POSITION_IN_THIS_REGION_START:
        // State description:  We are in destRegion!  Set the elevator and wrist to our target
        // Note that curRegion should always be destRegion if we arrived in this state

        elevator.setElevatorProfileTarget(destPosition.elevatorPosition);
        wrist.setWristAngle(destPosition.wristAngle);

        // Advance state to wait for wrist and elevator to get to target
        curState = MoveState.MOVE_TO_POSITION_IN_THIS_REGION_CONT;
        break;

      case MOVE_TO_POSITION_IN_THIS_REGION_CONT:
        // State description:  We are in destRegion!  Wait for the elevator and wrist to get to our target

        if ( (Math.abs(curWristAngle - destPosition.wristAngle) <= wristTargetTol) &&
             (Math.abs(curElevPos - destPosition.elevatorPosition) <= elevatorTargetTol) ) {
          curState = MoveState.DONE;
        }

        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (curState != MoveState.DONE) {
      // If there is an error (DONE_ERROR) of if this command is interrupted,
      // then stop the elevator and wrist at their current positions (or turn
      // off the motors).
      if (elevator.isElevatorCalibrated()) {
        elevator.setElevatorProfileTarget(elevator.getElevatorPosition());
      } else {
        elevator.stopElevatorMotors();
      }

      if (wrist.isWristCalibrated()) {
        wrist.setWristAngle(wrist.getWristAngle());
      } else {
        wrist.stopWrist();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return curState == MoveState.DONE || curState == MoveState.DONE_ERROR;
  }
}
