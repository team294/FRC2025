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
  /** <p>Drives to the nearest barge scoring location</p>
   *  
   *  If the robot is  aligned with the correct alliance barge then the robot will drive foward towards the barge while maintaining its current y value.
   * 
   * If the robot is not aligned with the correct alliance barge then the robot will drive foward while driving to the y value of the edge of the barge
   * 
   * <p> Command ends when robot reaches final position which if the driver continues to hold the button the robot can only be moved left to right</p>
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
