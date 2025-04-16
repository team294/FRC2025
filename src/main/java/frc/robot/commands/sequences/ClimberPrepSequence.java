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
   */
  public ClimberPrepSequence(Elevator elevator, Wrist wrist, Climber climber) {
    addCommands(
      sequence(
        parallel(
          new DataLogMessage(false, "ClimberPrepSequence Start"),
          runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CLIMBER_PREPPING)),
          // new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist),   // Do not move elevator/wrist, in case co-driver preps while scoring.  Co-driver is responsible for elevator/wrist.
          new ClimberSetRatchet(false, climber)
        ),
        new ClimberSetAngle(ClimberAngle.CLIMB_START, climber)
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CLIMBER_PREPPED)),
      new DataLogMessage(false, "ClimberPrepSequence End")
      );
  }
}
