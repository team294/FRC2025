// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotDimensions;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FileLogWrite;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.Field;

public class DriveToReefWithOdometryForCoral extends SequentialCommandGroup {
  /** <p>Drives to the closest reef scoring position against the reef</p>
   *  
   *  Before Joystick Use:
   *  If the distance to the final point is <= the robot's diameter AND the difference in angle between the current and final position is <= 30 degrees, it will drive to the scoring position.
   *  Otherwise, it will drive to a point offset (to allow for greater rotation) before driving to the scoring position.
   * 
   * <p></p>
   * 
   * After Joystick Use:
   * If the joystick is pushed in the direction opposite to the robot's relative position along the reef, the robot will switch to the other position on that side of the reef
   * (Robot on left position, joystick pushed right -> robot moves to the right position)
   * 
   * <p> Command ends when robot reaches final position (all jogs need to occur before getting to final position)
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   * @param rightJoystick
   * @param log FileLog utility
   */
  public DriveToReefWithOdometryForCoral(DriveTrain driveTrain, Field field, Joystick rightJoystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FileLogWrite(false, false, "DriveToReefWithOdometryForCoral", "Start"),
      
      sequence(
          race(
            //If the joystick is pushed opposite to the robot's position on the reef, this command ends (ending this command group)
            waitUntil(() -> ((-rightJoystick.getX() < -OIConstants.joystickJoggingDeadband && 
                      new Transform2d(field.getNearestAprilTagReef(driveTrain.getPose()), driveTrain.getPose()).getY() < 0) || 
                      (-rightJoystick.getX() > OIConstants.joystickJoggingDeadband && 
                      new Transform2d(field.getNearestAprilTagReef(driveTrain.getPose()), driveTrain.getPose()).getY() > 0))),

            //Drives to the nearest scoring position (which is on the wall), with an offset of half the robot's width plus a constant
            new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), 
                     new Transform2d((-RobotDimensions.robotWidth / 2.0) - DriveConstants.driveBackFromReefDistance, 0, new Rotation2d(0)))), 
                     0.02, 1, driveTrain)
          )
      ),

      //Joystick has been pushed opposite to the robot's position on the reef, determine the opposite reef position and switch to it
      either(
        sequence(
          //Drives to nearest left scoring position (offset off the wall half the robot's width plus a constant)
          new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), 
                        new Transform2d((-RobotDimensions.robotWidth / 2.0) - DriveConstants.driveBackFromReefDistance, 0, new Rotation2d(0)), true)), 
                        0.02, 1, driveTrain)
        ),
        either(
          sequence(
            //Drives to nearest right scoring position (offset off the wall half the robot's width a constant)
            new DriveToPose(CoordType.kAbsolute, () -> (field.getNearestReefScoringPositionWithOffset(driveTrain.getPose(), 
                        new Transform2d((-RobotDimensions.robotWidth / 2.0) - DriveConstants.driveBackFromReefDistance, 0, new Rotation2d(0)), false)), 
                        0.02, 1, driveTrain) 
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

      new FileLogWrite(false, false, "DriveToReefWithOdometryForCoral", "End")
    );
  }
}