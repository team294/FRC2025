// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HopperConstants;
import frc.robot.commands.CoralEffectorIntake;
import frc.robot.commands.HopperSetPercent;
import frc.robot.commands.HopperStopMotor;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.FileLog;


public class HopperCoralEffectorIntakeSequence extends SequentialCommandGroup {
  
  /**
   * Intakes coral by running Hopper and CoralEffector in parallel until the coral is safely in the CoralEffector.
   * @param hopper Hopper subsystem
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public HopperCoralEffectorIntakeSequence(Hopper hopper, CoralEffector coralEffector, FileLog log) {
    addCommands(
      new ParallelCommandGroup(
        new HopperSetPercent(HopperConstants.hopperIntakePercent, hopper, log),
        new CoralEffectorIntake(coralEffector, log)
      ),
      new HopperStopMotor(hopper, log)
    );
  }
}
