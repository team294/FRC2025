// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.StripEvents;
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
  public CoralIntakeSequence(Elevator elevator, Wrist wrist, Hopper hopper, CoralEffector coralEffector, LED led) {
    addCommands(
        sequence(
          new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.CORAL_ONLY, elevator, wrist),
          parallel(
            new HopperSetPercent(HopperConstants.intakePercent, hopper),
            new CoralEffectorIntakeEnhanced(coralEffector).raceWith(new LEDAnimationFlash(StripEvents.CORAL_INTAKING, led, LEDSegmentRange.StripAll))
          ).handleInterrupt(hopper::stopHopperMotor).alongWith(runOnce(() -> led.sendEvent(LED.StripEvents.NEUTRAL))),
          runOnce(() -> led.sendEvent(LED.StripEvents.CORAL_MODE)),
          new HopperStop(hopper)
        )
    );
  }
}
