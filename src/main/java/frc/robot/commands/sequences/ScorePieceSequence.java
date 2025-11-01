// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class ScorePieceSequence extends SequentialCommandGroup {
  /**
   * Scores a game piece. If holding algae (regardless of coral), score algae. Otherwise, score coral.
   * After scoring coral, backs up the robot.
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param driveTrain DriveTrain subsystem
   */
  public ScorePieceSequence(CoralEffector coralEffector, AlgaeGrabber algaeGrabber, DriveTrain driveTrain) {
    addCommands(
      new DataLogMessage(false, "ScorePieceSequence: Start"),
      either(
        new AlgaeGrabberOuttake(algaeGrabber),
        sequence( 
          new CoralEffectorOuttake(coralEffector)
          // new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
          //     0.5, 1.0, 
          //     TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
          //     true, true, driveTrain).asProxy()
        ), 
        () -> algaeGrabber.isAlgaePresent() || !coralEffector.isCoralPresent()
      ),

      new DataLogMessage(false, "ScorePieceSequence: End")
    );
  }
}