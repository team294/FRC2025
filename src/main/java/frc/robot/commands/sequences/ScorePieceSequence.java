// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ScorePieceSequence extends SequentialCommandGroup {
  /**
   * Scores a game piece. If holding algae (regardless of coral), score algae. Otherwise, score coral.
   * After scoring coral, backs up and lowers elevator and stows wrist
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param log FileLog utility
   */
  public ScorePieceSequence(CoralEffector coralEffector, AlgaeGrabber algaeGrabber, DriveTrain driveTrain, FileLog log) {
    addCommands(
      either(
        new AlgaeGrabberOuttake(algaeGrabber, log),
        sequence( 
          new CoralEffectorOuttake(coralEffector, log),
          new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain, log)
          // new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist, log)
        ), 
        () -> algaeGrabber.isAlgaePresent()
      )
    );
  }
}