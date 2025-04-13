// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutomatedDriveToReefAndIntakeAlgae extends SequentialCommandGroup {
  /**
   * Drives to nearest reef position and picks up algae (Algae level is manually input).
   * The robot will drive fully up against the reef, and then back up after picking up the algae.
   * @param algaeLevel the height of the algae, upper or lower
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param field Field field
   */
  public AutomatedDriveToReefAndIntakeAlgae(ElevatorWristPosition algaeLevel, DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      AlgaeGrabber algaeGrabber, Field field) {
    addCommands(
      new DataLogMessage(false, "AutomatedDriveToReefAndIntakeAlgae: Start"),
      
      parallel(
        sequence(
          // Move elevator 0.6 seconds after driving (only in auto)
          either(
            parallel(
              // Drive to nearest reef position
              new DriveToReefWithOdometryForAlgae(driveTrain, field),
              sequence(
                deadline(
                  waitSeconds(0.4),
                  new WristElevatorSafeMove(ElevatorWristPosition.CORAL_L1, RegionType.CORAL_ONLY, elevator, wrist)
                ),
                // Move elevator/wrist to correct position based on given level
                new AlgaeIntakeSequence(algaeLevel, elevator, wrist, algaeGrabber)
              )
            ),
            sequence(
              new DriveToReefWithOdometryForAlgae(driveTrain, field),
              new WristElevatorSafeMove(algaeLevel, RegionType.STANDARD, elevator, wrist)
            ),
            () -> DriverStation.isAutonomous()
          ),

          // Drive forward to get to the reef
          parallel(
            new DriveToPose(CoordType.kRelative, () -> new Pose2d(algaeLevel.equals(ElevatorWristPosition.ALGAE_LOWER) ? DriveConstants.distanceFromReefToPickupAlgaeLower : DriveConstants.distanceFromReefToPickupAlgaeUpper, 0, new Rotation2d(0)),
                0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain),

            // Intake algae
            new AlgaeIntakeSequence(algaeLevel, elevator, wrist, algaeGrabber).until(() -> algaeGrabber.isAlgaePresent())
          ),

          parallel(
            // Hold algae and back up
            new AlgaeGrabberSetPercent(0.1, algaeGrabber),
            new DriveToPose(CoordType.kRelative, () -> new Pose2d(-(algaeLevel.equals(ElevatorWristPosition.ALGAE_LOWER) ? DriveConstants.distanceFromReefToPickupAlgaeLower : DriveConstants.distanceFromReefToPickupAlgaeUpper), 0, Rotation2d.kZero),
                0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain)
          ).handleInterrupt(algaeGrabber::stopAlgaeGrabberMotor),

          parallel(
            new AlgaeGrabberStop(algaeGrabber),
            // Stow wrist
            new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist)
          )
        ),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.AUTO_DRIVE_IN_PROGRESS_REEF))
      ).handleInterrupt(
        () -> either(
          runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ALGAE_MODE)),
          runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
          () -> algaeGrabber.isAlgaePresent()
        )
      ),

      either(
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ALGAE_MODE)),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
        () -> algaeGrabber.isAlgaePresent()
      ),

      new DataLogMessage(false, "AutomatedDriveToReefAndIntakeAlgae: End")
    );
  }

  /**
   * Drives to nearest reef position and picks up algae (whether algae is higher vs. lower determined automatically).
   * The robot will drive fully up against the reef, and then back up after picking up the algae.
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param field Field field
   */
  public AutomatedDriveToReefAndIntakeAlgae(DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      AlgaeGrabber algaeGrabber, Field field) {
    addCommands(
      new DataLogMessage(false, "AutomatedDriveToReefAndIntakeAlgae: Start"),
      
      parallel(
        sequence(
          // Move elevator 0.6 seconds after driving (only in auto)
          either(
            parallel(
              // Drive to nearest reef position
              new DriveToReefWithOdometryForAlgae(driveTrain, field),
              sequence(
                deadline(
                  waitSeconds(0.4),
                  new WristElevatorSafeMove(ElevatorWristPosition.CORAL_L1, RegionType.CORAL_ONLY, elevator, wrist)
                ),
                // Move elevator/wrist to correct position based on given level
                new AlgaeIntakeSequence(field.getNearestAlgaeElevatorPosition(() -> driveTrain.getPose()), elevator, wrist, algaeGrabber)
              )
            ),
            sequence(
              new DriveToReefWithOdometryForAlgae(driveTrain, field),
              new WristElevatorSafeMove(field.getNearestAlgaeElevatorPosition(() -> driveTrain.getPose()), RegionType.STANDARD, elevator, wrist)
            ),
            () -> DriverStation.isAutonomous()
          ),

          // Drive forward to get to the reef
          parallel(
            sequence(
              new DriveToPose(CoordType.kRelative, () -> new Pose2d(field.getNearestAlgaeElevatorPosition(() -> driveTrain.getPose()).equals(ElevatorWristPosition.ALGAE_LOWER) ? DriveConstants.distanceFromReefToPickupAlgaeLower : DriveConstants.distanceFromReefToPickupAlgaeUpper, 0, new Rotation2d(0)),
              0.5, 1.0, 
              TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
              true, true, driveTrain)
            ),

            // Intake algae
            new AlgaeIntakeSequence(field.getNearestAlgaeElevatorPosition(() -> driveTrain.getPose()), elevator, wrist, algaeGrabber).until(() -> algaeGrabber.isAlgaePresent())
          ),

          parallel(
            // Hold algae and back up
            new AlgaeGrabberSetPercent(0.1, algaeGrabber),
            new DriveToPose(CoordType.kRelative, () -> new Pose2d(-(field.getNearestAlgaeElevatorPosition(() -> driveTrain.getPose()).equals(ElevatorWristPosition.ALGAE_LOWER) ? DriveConstants.distanceFromReefToPickupAlgaeLower : DriveConstants.distanceFromReefToPickupAlgaeUpper), 0, Rotation2d.kZero),
                0.5, 1.0, 
                TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
                true, true, driveTrain)
          ).handleInterrupt(algaeGrabber::stopAlgaeGrabberMotor),

          parallel(
            new AlgaeGrabberStop(algaeGrabber),
            // Stow wrist
            new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist)
          )
        ),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.AUTO_DRIVE_IN_PROGRESS_REEF))
      ).handleInterrupt(
        () -> either(
          runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ALGAE_MODE)),
          runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
          () -> algaeGrabber.isAlgaePresent()
        )
      ),
      
      either(
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ALGAE_MODE)),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
        () -> algaeGrabber.isAlgaePresent()
      ),

      new DataLogMessage(false, "AutomatedDriveToReefAndIntakeAlgae: End")
    );
  }
}
