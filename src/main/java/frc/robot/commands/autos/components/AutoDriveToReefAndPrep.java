// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.EnumMap;
import java.util.Map;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

public class AutoDriveToReefAndPrep extends SequentialCommandGroup {
  // Map to link ReefLevels to ElevatorPositions
  final static Map<ReefLevel, ElevatorWristPosition> reefToElevatorMap = new EnumMap<>(ReefLevel.class);
  static {
    reefToElevatorMap.put(ReefLevel.L1, ElevatorWristPosition.CORAL_L1);
    reefToElevatorMap.put(ReefLevel.L2, ElevatorWristPosition.CORAL_L2);
    reefToElevatorMap.put(ReefLevel.L3, ElevatorWristPosition.CORAL_L3);
    reefToElevatorMap.put(ReefLevel.L4, ElevatorWristPosition.CORAL_L4);
  }
  
  /**
   * Drives to reef location (using trajectory) and prepares elevator based on the reef level.
   * Runs intake command while driving to ensure coral is fully in, and then move the elevator after done driving.
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param trajectoryName TrajectoryName name of trajectory to follow 
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoDriveToReefAndPrep(ReefLevel level, TrajectoryName trajectoryName, DriveTrain driveTrain, Elevator elevator, Wrist wrist,
        CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance, TrajectoryCache cache) {
    addCommands(
      new DataLogMessage(false, "AutoDriveToReefAndPrep", "Init", "trajectoryName", trajectoryName.toString()),
      parallel(
        new CoralIntakeSequence(elevator, wrist, hopper, coralEffector),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(trajectoryName), driveTrain, alliance)
      ),
      new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist)
      // TODO add DriveToPose here, check these parameters
      // new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
      //         0.5, 1.0, 
      //         TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      //         true, true, driveTrain)
    );
  }

  /**
   * Drives to reef location and prepares elevator based on the reef level.
   * Runs intake command while driving to ensure coral is fully in, and then move the elevator after done driving.
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param fromHP true = starting at HP, false = starting at barge
   * @param end ReefLocation (A-L) to drive to
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoDriveToReefAndPrep(ReefLevel level, boolean fromHP, ReefLocation end, DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance, TrajectoryCache cache) {
    // Start from either HP or barge
    Trajectory<SwerveSample> trajectory = fromHP ? AutoSelection.getHPToReef(end) : AutoSelection.getBargeToReef(end);
    addCommands(
      new DataLogMessage(false, "AutoDriveToReefAndPrep", "Init", "trajectory", trajectory.name()),
      parallel(
        new CoralIntakeSequence(elevator, wrist, hopper, coralEffector),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectory, driveTrain, alliance)
      ),
      new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist)
      // TODO add DriveToPose here, check these parameters
      // new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
      //         0.5, 1.0, 
      //         TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      //         true, true, driveTrain)
    );
  }
}
