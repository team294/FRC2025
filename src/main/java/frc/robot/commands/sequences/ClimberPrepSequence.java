// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants.ClimberAngle;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.ClimberSetAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;


public class ClimberPrepSequence extends SequentialCommandGroup {
  /**
   * Prepares to latch on to the cage to climb by moving the wrist and elevator to the correct position,
   * and then moving the climber to the correct position.
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param climber Climber subsystem
   * @param log FileLog utility
   */
  public ClimberPrepSequence(Elevator elevator, Wrist wrist, Climber climber, FileLog log) {
    addCommands(
      new WristElevatorPrepSequence(ElevatorPosition.CORAL_HP, WristAngle.CORAL_HP, elevator, wrist, log),
      new ClimberSetAngle(ClimberAngle.CLIMB_START, climber, log)
    );
  }
}
