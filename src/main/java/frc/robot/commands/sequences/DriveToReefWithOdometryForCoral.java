// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.*;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.Constants.FieldConstants.ReefLocation;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class DriveToReefWithOdometryForCoral extends SequentialCommandGroup {
  /** 
   * Drives to the closest reef scoring position against the reef.
   * 
   * <p><b>Before Joystick Use:</b>
   * <p>If the distance to the final point is <= the robot's diameter AND the difference in angle between the current and final position is <= 30 degrees, it will drive to the scoring position.
   * Otherwise, it will drive to a point offset (to allow for greater rotation) before driving to the scoring position.
   * 
   * <p><b>After Joystick Use:</b>
   * <p>If the joystick is pushed in the direction opposite to the robot's relative position along the reef, the robot will switch to the other position on that side of the reef.
   * (Robot on left position, joystick pushed right -> robot moves to the right position)
   * 
   * <p>The sequence ends when robot reaches final position (all jogs need to occur before getting to final position).
   * @param level Reef level (1, 2, 3, 4)
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   * @param rightJoystick Right joystick
   */
  public DriveToReefWithOdometryForCoral(ReefLevel level, DriveTrain driveTrain, Field field, Joystick rightJoystick) {
    final double distance = (level == ReefLevel.L4) ? DriveConstants.distanceFromReefToScoreL4 : DriveConstants.distanceFromReefToScore;
    
    addCommands(
      new DataLogMessage(false, "DriveToReefWithOdometryForCoral", "Start"),
      
      sequence(
          race(
            //If the joystick is pushed opposite to the robot's position on the reef, this command ends (ending this command group)
            waitUntil(() -> ((-rightJoystick.getX() < -OIConstants.joystickJoggingDeadband && 
                      new Transform2d(field.getNearestAprilTagReef(driveTrain.getPose()), driveTrain.getPose()).getY() < 0) || 
                      (-rightJoystick.getX() > OIConstants.joystickJoggingDeadband && 
                      new Transform2d(field.getNearestAprilTagReef(driveTrain.getPose()), driveTrain.getPose()).getY() > 0))),

            // Drives to the nearest scoring position (which is on the wall), with an offset of half the robot's
            // width plus a constant (determined by scoring level, since L1 is different from L2, L3, and L4)
            either(
              new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), 
                new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)))), 
                0.02, 1, driveTrain),
              new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffsetL1(driveTrain.getPose(), 
                new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)))), 
                0.02, 1, driveTrain),
             () -> level != ReefLevel.L1
            )
          )
      ),

      //Joystick has been pushed opposite to the robot's position on the reef, determine the opposite reef position and switch to it
      either(
        sequence(
          // Drives to the nearest left position (on the wall), with an offset of half the robot's
          // width plus a constant (determined by scoring level, since L1 is different from L2, L3, and L4)
          either (
            new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), 
                        new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)), true)), 
                        0.02, 1, driveTrain),
            new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffsetL1(driveTrain.getPose(), 
                        new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)), true)), 
                        0.02, 1, driveTrain),
            () -> level != ReefLevel.L1
          )
        ),
        either(
          sequence(
            // Drives to the nearest right position (on the wall), with an offset of half the robot's
            // width plus a constant (determined by scoring level, since L1 is different from L2, L3, and L4)
            either (
              new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), 
                        new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)), false)), 
                        0.02, 1, driveTrain),
              new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffsetL1(driveTrain.getPose(), 
                        new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)), false)), 
                        0.02, 1, driveTrain),
              () -> level != ReefLevel.L1
            )
          ),
          sequence(
            // If you are at the same position the joystick is at, or the joystick is in the deadband (Which shouldn't occur, as these have to be false for this section to run), do nothing
          ), 
          // If joystick is right and you are at the left position
          () -> (-rightJoystick.getX() < -OIConstants.joystickJoggingDeadband && 
                  new Transform2d(field.getNearestAprilTagReef(driveTrain.getPose()), driveTrain.getPose()).getY() < 0)
        ),

        // If joystick is left and you are at the right position
        () -> (-rightJoystick.getX() > OIConstants.joystickJoggingDeadband && 
                  new Transform2d(field.getNearestAprilTagReef(driveTrain.getPose()), driveTrain.getPose()).getY() > 0)
      ),

      new DataLogMessage(false, "DriveToReefWithOdometryForCoral", "End")
    );
  }

  /**
   * For autonomous only
   * Drives to reef with odometry to the given reef position to the reef
   * @param level Reef level (1, 2, 3, 4)
   * @param location specific reef location to drive to 
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   */
  public DriveToReefWithOdometryForCoral(ReefLevel level, ReefLocation location, DriveTrain driveTrain, Field field) {
    final double distance = (level == ReefLevel.L4) ? DriveConstants.distanceFromReefToScoreL4 : DriveConstants.distanceFromReefToScore;
    
    addCommands(
      new DataLogMessage(false, "DriveToReefWithOdometryForCoral", "Start"),

      //Drives to the nearest scoring position (which is on the wall), with an offset of half the robot's width plus a constant
      either(
        new DriveToPose(CoordType.kAbsolute, () -> (field.getReefScoringPositionWithOffset(location, 
                new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)))), 
                0.02, 1, driveTrain),
        new DriveToPose(CoordType.kAbsolute, () -> (field.getReefScoringPositionWithOffsetL1(location, 
                new Transform2d((-RobotDimensions.robotWidth / 2.0) - distance, 0, new Rotation2d(0)))), 
                0.02, 1, driveTrain),
        () -> level != ReefLevel.L1
      ),
      
      new DataLogMessage(false, "DriveToReefWithOdometryForCoral", "End")
    );
  }
}