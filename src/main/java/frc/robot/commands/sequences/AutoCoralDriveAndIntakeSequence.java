// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants.ReefLocation;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;

public class AutoCoralDriveAndIntakeSequence extends SequentialCommandGroup {
  /**
   * Drive from start (reef location) to HP and intake a coral.
   * @param start ReefLocation (A-L) to start at
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param coralEffector EndEffector subsystem
   * @param alliance AllianceSelection alliance
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoCoralDriveAndIntakeSequence(ReefLocation start, DriveTrain driveTrain, Elevator elevator, CoralEffector coralEffector, 
      AllianceSelection alliance, TrajectoryCache cache, FileLog log) {
    addCommands(
      new AutoDriveToHPAndPrep(start, driveTrain, elevator, coralEffector, alliance, cache, log),
      //new EndEffectorCoralIntake(endEffector, log).withTimeout(2.5)
      new WaitCommand(.5)
    );
  }
}
