// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


/**
 * Prepares to score coral by moving the wrist and elevator to the indicated scoring position.
 * If the robot is holding algae, this sequence does nothing, since we cannot score coral while holding algae.
 * @param position position to move the elevator and wrist to (use ElevatorWwristConstants.ElevatorWristPosition)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param coralEffector CoralEffector subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class CoralScorePrepSequence extends SequentialCommandGroup {
  public CoralScorePrepSequence(ElevatorWristPosition position, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber, FileLog log) {
    addCommands(
      either(
        sequence(
          new WristElevatorSafeMove(position, RegionType.CORAL_ONLY, elevator, wrist, log),
          new WristSetAngle(position, wrist, log)
        ),
        none(),
        () -> !algaeGrabber.isAlgaePresent()
      )
    );
  }
}
