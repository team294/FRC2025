// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;


/**
 * Prepares to score coral by moving the wrist and elevator to the indicated scoring position.
 * If the robot is not holding coral, this sequence does nothing.
 * If the robot is holding coral, this sequence does nothing, since we cannot score coral while holding algae.
 * @param position position to move the elevator and wrist to (use ElevatorWwristConstants.ElevatorWristPosition)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param coralEffector CoralEffector subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class CoralScorePrepSequence extends SequentialCommandGroup {
  public CoralScorePrepSequence(ElevatorWristPosition position, Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, FileLog log) {
    addCommands(
      either(
        new WristElevatorPrepSequence(position, elevator, wrist, log),
        none(),
        () -> !algaeGrabber.isAlgaePresent() && coralEffector.isCoralPresent()
      )
    );
  }
}
