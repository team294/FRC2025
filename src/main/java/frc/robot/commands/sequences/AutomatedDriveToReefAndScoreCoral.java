// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import java.util.EnumMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.CoralEffectorOuttake;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.WristElevatorSafeMove;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;

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
   * @param rightJoystick Joystick joystick
   * @param field Field field
   * @param log FileLog log
   */
  public AutomatedDriveToReefAndScoreCoral(ReefLevel level, DriveTrain driveTrain, Elevator elevator, 
      Wrist wrist, CoralEffector coralEffector, Joystick rightJoystick, Field field, FileLog log) {
    
    addCommands(
      // Drive to nearest reef position
      new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick, log),

      // Move elevator/wrist to correct position based on given level
      new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist, log),

      // Drive forward to get to the reef (offset copied from DriveToReefWithOdometryForCoral and made positive)
      new DriveToPose(CoordType.kRelative, () -> new Pose2d((RobotDimensions.robotWidth / 2.0) + DriveConstants.driveBackFromReefDistance, 0, new Rotation2d(0)), 
        0.5, 1.0, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
        true, true, driveTrain, log),

      // Score piece
      new CoralEffectorOuttake(coralEffector, log),

      // Back up 
      new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero),
        0.5, 1.0, 
        TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
        true, true, driveTrain, log),

      // Move elevator/wrist to HP position
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.CORAL_ONLY, elevator, wrist, log)
    );
  }
}
