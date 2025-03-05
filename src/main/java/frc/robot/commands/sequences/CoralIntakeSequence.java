// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.CoralEffectorIntake;
import frc.robot.commands.HopperSetPercent;
import frc.robot.commands.HopperStop;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;


public class CoralIntakeSequence extends SequentialCommandGroup {
  
  /**
   * Intakes coral by moving the wrist and elevator to the intake position, and then 
   * running the hopper and coralEffector in parallel until the coral is safely in the coralEffector.
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param hopper Hopper subsystem
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralIntakeSequence(Elevator elevator, Wrist wrist, Hopper hopper, CoralEffector coralEffector, FileLog log) {
    addCommands(
      new WristElevatorPrepSequence(ElevatorPosition.CORAL_HP, WristAngle.CORAL_HP, elevator, wrist, log),
      new ParallelCommandGroup(
        new HopperSetPercent(HopperConstants.intakePercent, hopper, log),
        new CoralEffectorIntake(coralEffector, log)
      ),
      new HopperStop(hopper, log)
    );
  }
}
