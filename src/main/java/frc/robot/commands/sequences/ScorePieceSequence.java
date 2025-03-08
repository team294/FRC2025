// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.none;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeGrabberOuttake;
import frc.robot.commands.AlgaeGrabberStop;
import frc.robot.commands.CoralEffectorOuttake;
import frc.robot.commands.CoralEffectorStop;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class ScorePieceSequence extends SequentialCommandGroup {
  CoralEffector coralEffector;
  AlgaeGrabber algaeGrabber;
  FileLog log;

  /**
   * Scores piece depending on what piece(s) is/are held. If holding 
   * both coral and algae, score algae. Otherwise, score held piece.
   * If not holding any pieces, don't do anything.
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param log FileLog utility
   */
  public ScorePieceSequence(CoralEffector coralEffector, AlgaeGrabber algaeGrabber, FileLog log) {
    this.coralEffector = coralEffector;
    this.algaeGrabber = algaeGrabber;
    this.log = log;

    if (coralEffector.isCoralSafelyIn() && algaeGrabber.isAlgaePresent()) {
      addCommands(
        new AlgaeGrabberOuttake(algaeGrabber, log),
        new AlgaeGrabberStop(algaeGrabber, log)
      );
    }
    else if (coralEffector.isCoralSafelyIn() && !algaeGrabber.isAlgaePresent()) {
      addCommands(
        new CoralEffectorOuttake(coralEffector, log),
        new CoralEffectorStop(coralEffector, log)
      );
    }
    else if (!coralEffector.isCoralSafelyIn() && algaeGrabber.isAlgaePresent()) {
      addCommands(
        new AlgaeGrabberOuttake(algaeGrabber, log),
        new AlgaeGrabberStop(algaeGrabber, log)
      );
    }
    else addCommands(none());
  }
}