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


public class DriveToReefWithOdometryForAlgae extends SequentialCommandGroup {
  /**
   * Drives to the nearest algae pickup position on the reef with an offset.
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   * @param log FileLog utility
   */
  public DriveToReefWithOdometryForAlgae(DriveTrain driveTrain, Field field, DataLogUtil log) {
    addCommands(
      new FileLogWrite(false, false, "DriveToReefWithOdometryForAlgae", "Start", log),

      new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestAlgaePickupPositionWithOffset(driveTrain.getPose(), 
          new Transform2d((-RobotDimensions.robotWidth / 2.0) - DriveConstants.ReefAlgaePickupPositionOffset, 0, new Rotation2d(0)))),
          0.02, 1, driveTrain, log),

      new FileLogWrite(false, false, "DriveToReefWithOdometryForAlgae", "End", log)
    );
  }
}
