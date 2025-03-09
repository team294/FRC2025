// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.AlgaeGrabberIntake;
import frc.robot.commands.WristElevatorSafeMove;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


/**
 * Intakes algae by moving the wrist and elevator to the indicated intake position, and then 
 * running the algaeGrabber until the algae is in the mechanism.
 * @param position position to move elevator to (use ElevatorWristConstants.ElevatorWristPosition.ALGAE_...)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class AlgaeIntakeSequence extends SequentialCommandGroup {
  public AlgaeIntakeSequence(ElevatorWristPosition position, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber, FileLog log) {
    addCommands(
      new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist, log),
      new AlgaeGrabberIntake(algaeGrabber, log)    
    );
  }
}
