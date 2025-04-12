// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.Constants.FieldConstants.ReefLocation;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutomatedDriveToReefAndScoreCoral extends SequentialCommandGroup {
  // Map to link ReefLevels to ElevatorPositions
  final static Map<ReefLevel, ElevatorWristPosition> reefToElevatorMap = new EnumMap<>(ReefLevel.class);
  static {
    reefToElevatorMap.put(ReefLevel.L1, ElevatorWristPosition.CORAL_L1);
    reefToElevatorMap.put(ReefLevel.L2, ElevatorWristPosition.CORAL_L2);
    reefToElevatorMap.put(ReefLevel.L3, ElevatorWristPosition.CORAL_L3);
    reefToElevatorMap.put(ReefLevel.L4, ElevatorWristPosition.CORAL_L4);
  }
  /**
   * Drives to nearest reef position and scores coral in given level.
   * If not scoring on L4, the robot will drive fully up against the reef, and then back up after scoring.
   * If scoring on L4, the robot will score while not fully up against the reef, and the routine ends when the piece is scored.
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param rightJoysitck Right joystick
   * @param field Field field
   */
  public AutomatedDriveToReefAndScoreCoral(ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, 
      AlgaeGrabber algaeGrabber, Joystick rightJoystick, Field field) {
    addCommands(
      parallel(
        sequence(
          new DataLogMessage(false, "AutomatedDriveToReefAndScoreCoral: Start"),
          // Move elevator 0.6 seconds after driving (only in auto)
          either(
            parallel(
              // Drive to nearest reef position
              new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick),
              sequence(
                deadline(
                  waitSeconds(0.4),
                  sequence(
                    waitUntil(() -> coralEffector.getHoldMode()),
                    new WristElevatorSafeMove(ElevatorWristPosition.CORAL_L1, RegionType.CORAL_ONLY, elevator, wrist)
                  )
                ),
                // Move elevator/wrist to correct position based on given level
                new CoralScorePrepSequence(reefToElevatorMap.get(level), elevator, wrist, algaeGrabber, coralEffector)
              )
            ),
            sequence(
              new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick),
              new CoralScorePrepSequence(reefToElevatorMap.get(level), elevator, wrist, algaeGrabber, coralEffector)
            ),
            () -> DriverStation.isAutonomous()
          ),

          // If not scoring on L4, drive forward to get to the reef
          either(
            new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.distanceFromReefToScore, 0, new Rotation2d(0)),
                0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain),
            none(),
            () -> level == ReefLevel.L1
          ),

          // Score piece
          new CoralEffectorOuttake(coralEffector),

          // If scoring on L1, wait 0.5 seconds before backing up
          either(waitSeconds(0.5), none(), () -> level == ReefLevel.L1),

          // If not scoring on L4, back up
          either(
            new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.distanceFromReefToScore, 0, Rotation2d.kZero),
                0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain),
            none(),
            () -> level == ReefLevel.L1 
          )
        ),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.AUTO_DRIVE_IN_PROGRESS_REEF))
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),

      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),

      new DataLogMessage(false, "AutomatedDriveToReefAndScoreCoral: End")
    );
  }

  /**
   * AutomatedDriveToReefAndScoreCoral for autonomous only
   * If not scoring on L4, the robot will drive fully up against the reef, and then back up after scoring.
   * If scoring on L4, the robot will score while not fully up against the reef, and the routine ends when the piece is scored.
   * @param location Reef location to score on
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param rightJoystick Right joystick
   * @param field Field field
   */
  public AutomatedDriveToReefAndScoreCoral(ReefLocation location, boolean isLastCoral, ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, 
      AlgaeGrabber algaeGrabber, Hopper hopper, Joystick rightJoystick, Field field) {
    addCommands(
      new DataLogMessage(false, "AutomatedDriveToReefAndScoreCoral: Start"),
      parallel(
        sequence(
          parallel(
            // Drive to specific reef location position
            new DriveToReefWithOdometryForCoral(location, driveTrain, field), 
            
            sequence(
              // If the endeffector is in hold mode (Coral is safely in intake) then do nothing, otherwise intake the coral until the piece is present
              either(
                none(), 
                new CoralIntakeSequence(elevator, wrist, hopper, coralEffector),
                () -> (coralEffector.getHoldMode())),
    
              deadline(
                // Move elevator for 0.6 seconds after driving + coral is fully intaked
                waitSeconds(0.6),
                new WristElevatorSafeMove(ElevatorWristPosition.CORAL_L1, RegionType.CORAL_ONLY, elevator, wrist)
              ),

              // Move elevator/wrist to correct position based on given level, only if isLastCoral is not true (meaning we are not at the last coral so we still want to move elevator)
              either(
                new CoralScorePrepSequence(reefToElevatorMap.get(level), elevator, wrist, algaeGrabber, coralEffector),
                none(),
                () -> !isLastCoral
            )
          ),
        
          // If scoring on L1, drive forward to get to the reef
          either(
            new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.distanceFromReefToScore, 0, new Rotation2d(0)),
                0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain),
            none(),
            () -> level == ReefLevel.L1
          ),
    
          // Score piece (only if it is not the last coral location)
          either(
            new CoralEffectorOuttake(coralEffector),
            none(),
            () -> !isLastCoral
          ),
    
          // If scoring on L1, wait 0.5 seconds then back up
          either(
            sequence(
              waitSeconds(0.5),
              new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.distanceFromReefToScore, 0, Rotation2d.kZero),
                  0.5, 1.0, 
                  TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                  true, true, driveTrain)
            ),
            none(),
            () -> level == ReefLevel.L1  
          )    
        ),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.AUTO_DRIVE_IN_PROGRESS_REEF))
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),     
      
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL))),

      new DataLogMessage(false, "AutomatedDriveToReefAndScoreCoral: End")
    );
  }

}
