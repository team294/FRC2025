// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import java.util.EnumMap;
import java.util.Map;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

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
   * Drives to nearest reef position, scores coral in given level, backs up by 0.25 meters (by driveBackFromReefDistance, which is the start/end position of choreo reef trajectories)
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
      // Drive to nearest reef position
      new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick),

      // Move elevator/wrist to correct position based on given level
      new CoralScorePrepSequence(reefToElevatorMap.get(level), elevator, wrist, algaeGrabber),

      // Drive forward to get to the reef (only if not scoring on L4)
      either(
        new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.L4DriveinDistance, 0, new Rotation2d(0)),
        0.5, 1.0, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
        true, true, driveTrain),
        new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.driveBackFromReefDistance, 0, new Rotation2d(0)),
        0.5, 1.0, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
        true, true, driveTrain),
        () -> level == ReefLevel.L4  
      ),

      // Score piece
      new CoralEffectorOuttake(coralEffector),

      // If scoring on L1, wait 0.5 seconds before backing up
      either(waitSeconds(0.5), none(), () -> level == ReefLevel.L1),

      // Back up (only if not scoring on L4)
      either(
        new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.L4DriveinDistance, 0, new Rotation2d(0)),
        0.5, 1.0, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
        true, true, driveTrain),
        new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero),
        0.5, 1.0, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
        true, true, driveTrain),
        () -> level == ReefLevel.L4  
      )
    );
  }
}
