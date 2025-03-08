// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;


/**
 * Prepares to score algae by moving the wrist and elevator to the indicated scoring position.
 * If the robot is not holding algae, this sequence does nothing.
 * @param position position to move elevator to (use ElevatorConstants.ElevatorPosition.ALGAE_...)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class AlgaeScorePrepSequence extends SequentialCommandGroup {
  public AlgaeScorePrepSequence(ElevatorPosition position, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber, FileLog log) {
    WristAngle angle = null;

    switch (position) {
      case ALGAE_PROCESSOR:
        angle = WristAngle.ALGAE_PROCESSOR;
        break;
      case ALGAE_NET:
        angle = WristAngle.ALGAE_NET;
        break;
      default:
        break;
    }

    if (angle == null) {
      addCommands(none());
    } else {
      addCommands(
        either(
          new WristElevatorPrepSequence(position, angle, elevator, wrist, log),
          none(),
          () -> algaeGrabber.isAlgaePresent()
        )
      );
    }
  }
}
