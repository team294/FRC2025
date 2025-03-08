// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class ScorePieceSequence extends Command {
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
  }

  public void initialize() {
    if (coralEffector.isCoralSafelyIn() && algaeGrabber.isAlgaePresent()) new AlgaeGrabberOuttake(algaeGrabber, log);
    else if (coralEffector.isCoralSafelyIn() && !algaeGrabber.isAlgaePresent()) new CoralEffectorOuttake(coralEffector, log);
    else if (!coralEffector.isCoralSafelyIn() && algaeGrabber.isAlgaePresent()) new AlgaeGrabberOuttake(algaeGrabber, log);
  }

  public void execute() {
}

  public void end(boolean interrupted) {
    parallel(
      new CoralEffectorStop(coralEffector, log),
      new AlgaeGrabberStop(algaeGrabber, log)
    );
  }

  public boolean isFinished() {
    return true;
  }
}
