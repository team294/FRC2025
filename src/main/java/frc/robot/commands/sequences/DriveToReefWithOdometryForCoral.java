// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

public class DriveToReefWithOdometryForCoral extends SequentialCommandGroup {

  /**
   * Drives to the closest coral scoring position on the reef using odometry.
   * The robot will first drive toward the position but have some offset, to allow rotation, and then drive straight up against the reef.
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   * @param log FileLog utility
   */
  public DriveToReefWithOdometryForCoral(DriveTrain driveTrain, Field field, FileLog log) {
    addCommands(
      // Drives to the nearest coral scoring position, with an offset of half the robot's diameter plus 5cm
      new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), new Transform2d((-RobotDimensions.robotDiagonal / 2.0) - 0.05, 0, new Rotation2d(0)))), 0.03, 3, driveTrain, log),
      
      // Drives the remaining distance into the reef
      new DriveToPose(CoordType.kRelative, () -> (new Pose2d(((RobotDimensions.robotDiagonal-RobotDimensions.robotWidth) / 2.0) + 0.05, 0, new Rotation2d(0))), TrajectoryConstants.maxPositionErrorMeters, 3, driveTrain, log)
    );
  }
}