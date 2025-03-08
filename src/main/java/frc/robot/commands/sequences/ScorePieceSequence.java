// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeGrabberOuttake;
import frc.robot.commands.CoralEffectorOuttake;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class ScorePieceSequence extends SequentialCommandGroup {
  /**
   * Scores a game piece. If holding algae (regardless of coral), score algae. Otherwise, score coral.
   * Does not move the elevator or wrist.
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param log FileLog utility
   */
  public ScorePieceSequence(CoralEffector coralEffector, AlgaeGrabber algaeGrabber, FileLog log) {
    addCommands(
      either(
        new AlgaeGrabberOuttake(algaeGrabber, log), 
        new CoralEffectorOuttake(coralEffector, log), 
        () -> algaeGrabber.isAlgaePresent()
      )
    );
  }
}