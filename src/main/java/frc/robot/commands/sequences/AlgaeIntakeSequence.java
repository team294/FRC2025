// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.commands.AlgaeGrabberIntake;
import frc.robot.commands.ElevatorSetPosition;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;

/**
 * Intakes algae by moving the elevator to the correct intake
 * position based on where algae is being picked up from 
 * TODO add wrist movement
 * @param elevator Elevator subsystem
 * @param algaeGrabber Algae Grabber subsystem
 * @param position ElevatorPosition constant to move elevator to
 * @param log FileLog utility
 */
public class AlgaeIntakeSequence extends SequentialCommandGroup {
  /** Creates a new AlgaeIntakeSequence. */
  public AlgaeIntakeSequence(Elevator elevator, /*Wrist wrist,*/ AlgaeGrabber algaeGrabber, ElevatorPosition position, FileLog log) {
    addCommands(
      new ElevatorSetPosition(position.value, elevator, log),
      new AlgaeGrabberIntake(algaeGrabber, log)
    );
  }
}
