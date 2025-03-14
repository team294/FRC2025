// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoCoralDriveAndScoreSequence extends SequentialCommandGroup {
  /**
   * Drive from barge to end (reef location) and score a coral.
   * @param fromHP true = starting at HP, false = starting at barge
   * @param end ReefLocation (A-L) to end at
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param alliance AllianceSelection alliance
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoCoralDriveAndScoreSequence(boolean fromHP, ReefLocation end, ReefLevel level, DriveTrain driveTrain,
      Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, AllianceSelection alliance,
      TrajectoryCache cache, FileLog log) {
    addCommands(
      new AutoDriveToReefAndPrep(level, fromHP, end, driveTrain, elevator, wrist, coralEffector, hopper, alliance, cache, log),
      new ScorePieceSequence(coralEffector, algaeGrabber, driveTrain, log),
      waitSeconds(0.5)
    );
  }
}
