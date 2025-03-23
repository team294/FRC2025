// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ClimberConstants.ClimberAngle;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


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
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.CORAL_ONLY, elevator, wrist),
      new ClimberSetAngle(ClimberAngle.CLIMB_START, climber)
    );
  }
}
