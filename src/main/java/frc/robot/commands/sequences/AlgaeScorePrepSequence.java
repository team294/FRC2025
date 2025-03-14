// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


/**
 * Prepares to score algae by moving the wrist and elevator to the indicated scoring position.
 * If the robot is not holding algae, this sequence does nothing.
 * @param position position to move elevator to (use ElevatorWristConstants.ElevatorWristPosition.ALGAE_...)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class AlgaeScorePrepSequence extends SequentialCommandGroup {
  public AlgaeScorePrepSequence(ElevatorWristPosition position, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber, FileLog log) {
    addCommands(
      either(
        new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist, log),
        none(),
        () -> algaeGrabber.isAlgaePresent()
      )
    );
  }
}
