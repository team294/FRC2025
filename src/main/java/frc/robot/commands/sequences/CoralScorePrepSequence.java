// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.commands.ElevatorSetPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

/**
 * Prepares to score coral by moving the elevator to the indicated scoring position.
 * TODO add wrist movement
 * @param position position to move elevator to (use ElevatorConstants.ElevatorPosition.CORAL_...)
 * @param elevator Elevator subsystem
 * @param log FileLog utility
 */
public class CoralScorePrepSequence extends SequentialCommandGroup {
  public CoralScorePrepSequence(Elevator elevator, /*Wrist wrist,*/ ElevatorPosition position, FileLog log) {
    addCommands(
      new ElevatorSetPosition(position.value, elevator, log)
      // TODO set in coral mode
    );
  }
}
