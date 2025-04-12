// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ClimberConstants.ClimberAngle;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.LEDEventUtil;


public class ClimberPrepSequence extends SequentialCommandGroup {
  /**
   * Prepares to latch on to the cage to climb by moving the wrist and elevator to the correct position,
   * and then moving the climber to the correct position.
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param climber Climber subsystem
   * @param log FileLog utility
   */
  public ClimberPrepSequence(Elevator elevator, Wrist wrist, Climber climber) {
    addCommands(
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      parallel(
        new ClimberSetAngle(ClimberAngle.CLIMB_START, climber),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CLIMBER_PREPPING))
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CLIMBER_PREPPED))
    );
  }
}
