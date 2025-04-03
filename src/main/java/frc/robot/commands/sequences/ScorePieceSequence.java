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
   * @param led LED subsystem
   * @param log FileLog utility
   */
  public ScorePieceSequence(CoralEffector coralEffector, AlgaeGrabber algaeGrabber, DriveTrain driveTrain, LED led) {
    addCommands(
      either(
        new AlgaeGrabberOuttake(algaeGrabber).andThen(
          either( // if coral is present after outtaking algae, send CORAL_MODE event
            runOnce(() -> led.sendEvent(LED.StripEvents.CORAL_MODE)),
            runOnce(() -> led.sendEvent(LED.StripEvents.NEUTRAL)),
            () -> coralEffector.isCoralPresent()
          )
        ),
        sequence( 
          new CoralEffectorOuttake(coralEffector),
          runOnce(() -> led.sendEvent(LED.StripEvents.NEUTRAL))
          // new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
          //     0.5, 1.0, 
          //     TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
          //     true, true, driveTrain).asProxy()
        ), 
        () -> algaeGrabber.isAlgaePresent() || !coralEffector.isCoralPresent()
      )
    );
  }
}