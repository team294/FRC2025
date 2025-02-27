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
 * Moves elevator to correct position to score algae based on ElevatorPosition constant
 * TODO add wrist movement
 * @param elevator Elevator subsystem
 * @param position ElevatorPosition constant to move elevator to
 * @param log FileLog utility
 */
public class AlgaeScorePrepSequence extends SequentialCommandGroup {
  /** Creates a new AlgaeScorePrepSequence. */
  public AlgaeScorePrepSequence(Elevator elevator, /*Wrist wrist,*/ ElevatorPosition position, FileLog log) {
    addCommands(
      new ElevatorSetPosition(position.value, elevator, log)
    );
  }
}
