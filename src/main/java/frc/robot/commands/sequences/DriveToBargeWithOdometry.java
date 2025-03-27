// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;


public class DriveToBargeWithOdometry extends SequentialCommandGroup {
  /**
   * Drives to the nearest point from which we can score in the barge
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   * @param log FileLog utility
   */
  public DriveToBargeWithOdometry(DriveTrain driveTrain, Field field) {
    addCommands(

      new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestBargeScoringPosition(driveTrain.getPose())), 0.02, 1, driveTrain)

    );
  }
}
