// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.ReefLocation;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoCoralDriveAndIntakeSequence extends SequentialCommandGroup {
  /**
   * Drive from start (reef location) to HP and intake a coral.
   * @param start ReefLocation (A-L) to start at
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection alliance
   * @param log FileLog log
   */
  public AutoCoralDriveAndIntakeSequence(ReefLocation start, DriveTrain driveTrain, Elevator elevator, Wrist wrist,
      CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance) {
    addCommands(
      new DataLogMessage(false, "AutoCoralDriveAndIntakeSequence: Start, starting reef location =", start.toString()),
      new AutoDriveToHPAndPrep(start, driveTrain, elevator, wrist, coralEffector, alliance),
      new CoralIntakeSequence(elevator, wrist, hopper, coralEffector), //.withTimeout(2.5)
      new DataLogMessage(false, "AutoCoralDriveAndIntakeSequence: End")
    );
  }
}
