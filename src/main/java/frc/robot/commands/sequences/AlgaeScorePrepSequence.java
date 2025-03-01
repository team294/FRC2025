// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.ElevatorSetPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;


/**
 * Prepares to score algae by moving the elevator to the indicated scoring position.
 * @param position position to move elevator to (use ElevatorConstants.ElevatorPosition.ALGAE_...)
 * @param elevator Elevator subsystem
 * @param log FileLog utility
 */
public class AlgaeScorePrepSequence extends SequentialCommandGroup {
  public AlgaeScorePrepSequence(ElevatorPosition position, Elevator elevator, Wrist wrist, FileLog log) {
    addCommands(
      new ElevatorSetPosition(position.value, elevator, log),
      new WristSetAngle(WristAngle.algaeScore, wrist, log)
    );
  }
}
