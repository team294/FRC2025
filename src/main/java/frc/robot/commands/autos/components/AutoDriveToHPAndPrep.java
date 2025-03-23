// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.ReefLocation;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

public class AutoDriveToHPAndPrep extends SequentialCommandGroup {
  /**
   * Drives to HP (using trajectory) and prepares for intaking coral.
   * Moves the elevator in parallel while driving.
   * @param trajectoryName TrajectoryName name of trajectory to follow 
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoDriveToHPAndPrep(TrajectoryName trajectoryName, DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, 
          AllianceSelection alliance, TrajectoryCache cache) {    
    addCommands(
      new FileLogWrite(false, false, "AutoDriveToHPAndPrep", "Init", "trajectoryName", trajectoryName.toString()),
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist),
      new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(trajectoryName), driveTrain, alliance),
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist)
    );
  }

  /**
   * Drives to HP (using start = reef location) and prepares for intaking coral.
   * Moves the elevator in parallel while driving.
   * @param start ReefLocation (A-L) to start at
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoDriveToHPAndPrep(ReefLocation start, DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, 
          AllianceSelection alliance, TrajectoryCache cache) {
    addCommands(
      new FileLogWrite(false, false, "AutoDriveToHPAndPrep", "Init", "startingReefLocation", start.toString()),
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist),
      new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, AutoSelection.getReefToHP(start), driveTrain, alliance),
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist)
    );
  }
}
