// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

/**
 * Prepares to score coral by moving the wrist and elevator to the indicated scoring position.
 * @param position position to move elevator to (use ElevatorConstants.ElevatorPosition.CORAL_...)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param log FileLog utility
 */
public class CoralScorePrepSequence extends SequentialCommandGroup {
  public CoralScorePrepSequence(Elevator elevator, Wrist wrist, ElevatorPosition position, FileLog log) {
    WristAngle angle = 
      position == ElevatorPosition.CORAL_L1 ? WristAngle.CORAL_L1 : 
      position == ElevatorPosition.CORAL_L2 || position == ElevatorPosition.CORAL_L2 ? WristAngle.CORAL_L2_L3 : 
      position == ElevatorPosition.CORAL_L4 ? WristAngle.CORAL_L4 :
      null;
    if (angle == null) return;

    addCommands(
      new WristElevatorPrepSequence(position, angle, elevator, wrist, log)
      // TODO set in coral mode
    );
  }
}
