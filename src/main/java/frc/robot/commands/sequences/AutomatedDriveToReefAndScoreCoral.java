// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.EnumMap;
import java.util.Map;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.StripEvents;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutomatedDriveToReefAndScoreCoral extends SequentialCommandGroup {
  // Map to link ReefLevels to ElevatorPositions
  final static Map<ReefLevel, ElevatorWristPosition> reefToElevatorMap = new EnumMap<>(ReefLevel.class);
  static {
    reefToElevatorMap.put(ReefLevel.L1, ElevatorWristPosition.CORAL_L1);
    reefToElevatorMap.put(ReefLevel.L2, ElevatorWristPosition.CORAL_L2);
    reefToElevatorMap.put(ReefLevel.L3, ElevatorWristPosition.CORAL_L3);
    reefToElevatorMap.put(ReefLevel.L4, ElevatorWristPosition.CORAL_L4);
  }
  /**
   * Drives to nearest reef position and scores coral in given level.
   * If not scoring on L4, the robot will drive fully up against the reef, and then back up after scoring.
   * If scoring on L4, the robot will score while not fully up against the reef, and the routine ends when the piece is scored.
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param led LED subsystem
   * @param rightJoysitck Right joystick
   * @param field Field field
   */
  public AutomatedDriveToReefAndScoreCoral(ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, 
      AlgaeGrabber algaeGrabber, LED led, Joystick rightJoystick, Field field) {
    addCommands(
      new DataLogMessage(false, "AutomatedDriveToReefAndScoreCoral: Start"),
      
      // Move elevator 0.6 seconds after driving (only in auto)
      either(
        parallel(
          // Drive to nearest reef position
          new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick),
          sequence(
            deadline(
              waitSeconds(0.4),
              waitUntil(() -> coralEffector.getHoldMode()),
              new WristElevatorSafeMove(ElevatorWristPosition.CORAL_L1, RegionType.CORAL_ONLY, elevator, wrist)
            ),
            // Move elevator/wrist to correct position based on given level
            new CoralScorePrepSequence(reefToElevatorMap.get(level), elevator, wrist, algaeGrabber, coralEffector)
          )
        ),
        sequence(
          new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick),
          new CoralScorePrepSequence(reefToElevatorMap.get(level), elevator, wrist, algaeGrabber, coralEffector)
        ),
        () -> DriverStation.isAutonomous()
      ),

      // If not scoring on L4, drive forward to get to the reef
      either(
        new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.distanceFromReefToScore, 0, new Rotation2d(0)),
            0.5, 1.0, 
            TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
            true, true, driveTrain),
        none(),
        () -> level == ReefLevel.L1
      ),

      // Score piece
      new CoralEffectorOuttake(coralEffector, led),

      // If scoring on L1, wait 0.5 seconds before backing up
      either(waitSeconds(0.5), none(), () -> level == ReefLevel.L1),

      // If not scoring on L4, back up
      either(
        new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.distanceFromReefToScore, 0, Rotation2d.kZero),
            0.5, 1.0, 
            TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
            true, true, driveTrain),
        none(),
        () -> level == ReefLevel.L1 
      ),//.raceWith(new LEDAnimationRainbow(led, LEDSegmentRange.StripAll)),

      // runOnce(() -> led.sendEvent(StripEvents.AUTO_DRIVE_COMPLETE)),

      new DataLogMessage(false, "AutomatedDriveToReefAndScoreCoral: End")
    );
  }
}
