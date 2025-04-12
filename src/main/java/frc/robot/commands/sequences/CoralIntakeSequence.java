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
import frc.robot.utilities.LEDEventUtil;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


public class CoralIntakeSequence extends SequentialCommandGroup {
  
  /**
   * Intakes coral by moving the wrist and elevator to the intake position, and then 
   * running the hopper and coralEffector in parallel until the coral is safely in the coralEffector.
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param hopper Hopper subsystem
   * @param coralEffector CoralEffector subsystem
   * @param led LED subsystem
   */
  public CoralIntakeSequence(Elevator elevator, Wrist wrist, Hopper hopper, CoralEffector coralEffector) {
    addCommands(
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.CORAL_ONLY, elevator, wrist),
      parallel(
        new HopperSetPercent(HopperConstants.intakePercent, hopper),
        parallel(
          sequence(
            new CoralEffectorIntakeEnhanced(coralEffector),
            new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist)
          ),
          runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CORAL_INTAKING))
        ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL))
      ).handleInterrupt(hopper::stopHopperMotor),
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      either(
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CORAL_MODE)),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
        () -> coralEffector.isCoralPresent()),
      new HopperStop(hopper)
    );
  }
}
