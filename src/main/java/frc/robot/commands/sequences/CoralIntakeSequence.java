// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


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
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.CORAL_ONLY, elevator, wrist, log),
      parallel(
        new HopperSetPercent(HopperConstants.intakePercent, hopper, log),
        new CoralEffectorIntakeEnhanced(coralEffector, log)
      ).handleInterrupt(hopper::stopHopperMotor),
      new HopperStop(hopper, log)
    );
  }
}
