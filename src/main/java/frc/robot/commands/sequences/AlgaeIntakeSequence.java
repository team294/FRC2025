// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.AlgaeGrabberIntake;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;


/**
 * Intakes algae by moving the wrist and elevator to the indicated intake position, and then 
 * running the algaeGrabber until the algae is in the mechanism.
 * @param position position to move elevator to (use ElevatorConstants.ElevatorPosition.ALGAE_...)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class AlgaeIntakeSequence extends SequentialCommandGroup {
  public AlgaeIntakeSequence(ElevatorPosition position, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber, FileLog log) {
    WristAngle angle = 
      position == ElevatorPosition.ALGAE_GROUND ? WristAngle.ALGAE_GROUND
      : position == ElevatorPosition.ALGAE_LOWER || position == ElevatorPosition.ALGAE_UPPER ? WristAngle.ALGAE_REEF
      : null;

    addCommands(
      either(
        sequence(
          new WristElevatorPrepSequence(position, angle, elevator, wrist, log),
          new AlgaeGrabberIntake(algaeGrabber, log)    
        ),
        none(),
        () -> angle != null
      )
    );
  }
}
