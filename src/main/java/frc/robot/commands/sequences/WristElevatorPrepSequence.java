// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.ElevatorSetPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.ElevatorWristRegions;
import frc.robot.utilities.FileLog;

public class WristElevatorPrepSequence extends SequentialCommandGroup {

  /**
   * Moves the wrist and elevator in sequence, accounting for interlocks and regions.
   * TODO add interlocks and regions
   * @param position position to move the elevator and wrist to (use ElevatorWwristConstants.ElevatorWristPosition)
   * @param type  Region type = CORAL_ONLY or STANDARD
   * @param elevatorWristRegions ElevatorWristRegions calculator
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristElevatorPrepSequence(ElevatorWristPosition position, 
            ElevatorWristRegions.RegionType type, ElevatorWristRegions elevatorWristRegions,
            Elevator elevator, Wrist wrist, FileLog log) {
    addCommands(
      new WristSetAngle(position.wristAngle, wrist, log),
      new ElevatorSetPosition(position.elevatorPosition, elevator, log)
    );
  }
}
