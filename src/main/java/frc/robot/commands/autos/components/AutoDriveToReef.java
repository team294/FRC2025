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

public class AutoDriveToReef extends SequentialCommandGroup {
  /** Below constructor is outdated and not used anywhere
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
   */ /* 
  public AutoDriveToReef(ReefLevel level, TrajectoryName trajectoryName, DriveTrain driveTrain, Elevator elevator, Wrist wrist,
        CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance, TrajectoryCache cache, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoDriveToReefAndPrep", "Init", log, "trajectoryName", trajectoryName.toString()),
      parallel(
        new CoralIntakeSequence(elevator, wrist, hopper, coralEffector, log),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(trajectoryName), driveTrain, alliance, log)
      ),
      new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist, log)
      // TODO add DriveToPose here, check these parameters
      // new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
      //         0.5, 1.0, 
      //         TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      //         true, true, driveTrain, log)
    );
  } */

  /**
   * Drives to reef location, trajectory gets cut off early to allow for smooth transition into AutomatedDriveToReefAndScoreCoral (does not move elevator)
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
  public AutoDriveToReef(ReefLevel level, boolean fromHP, ReefLocation end, DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance, TrajectoryCache cache, Field field, FileLog log) {

    // Choose trajectory to drive (Start from either HP or barge)
    Trajectory<SwerveSample> trajectory = fromHP ? AutoSelection.getHPToReef(end) : AutoSelection.getBargeToReef(end);
    addCommands(
      new FileLogWrite(false, false, "AutoDriveToReefAndPrep", "Init", log, "trajectory", trajectory.name()),
      parallel(
        new CoralIntakeSequence(elevator, wrist, hopper, coralEffector, log),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectory, driveTrain, alliance, log)
      )// TODO make trajectory cut itself off (timeout = get time - 0.3 ish seconds, use .withTimeout(timeout))




      //new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist, log)

      // Drive to the given reef position 
      // new DriveToPose(CoordType.kAbsolute, () -> (field.getRobotReefScoringPosition(end)), 0.5, 1.0, TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, true, true, driveTrain, log)
    );
  }
}
