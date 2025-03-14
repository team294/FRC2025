// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.FieldConstants.ReefLocation;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.AutoSelection;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

public class AutoDriveToHPAndPrep extends SequentialCommandGroup {
  /**
   * Drives to HP (using trajectory) and prepares for intaking coral.
   * Moves the elevator in parallel while driving.
   * @param trajectoryName TrajectoryName name of trajectory to follow 
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param coralEffector EndEffector subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoDriveToHPAndPrep(TrajectoryName trajectoryName, DriveTrain driveTrain, Elevator elevator, CoralEffector coralEffector, 
          AllianceSelection alliance, TrajectoryCache cache, FileLog log) {    
    addCommands(
      new FileLogWrite(false, false, "AutoDriveToHPAndPrep", "Init", log, "trajectoryName", trajectoryName.toString()),
      new ParallelCommandGroup(
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(trajectoryName), driveTrain, alliance, log),
        new ElevatorSetPosition(ElevatorPosition.CORAL_HP.value, elevator, coralEffector, log)    
      )
    );
  }

  /**
   * Drives to HP (using start = reef location) and prepares for intaking coral.
   * Moves the elevator in parallel while driving.
   * @param start ReefLocation (A-L) to start at
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param coralEffector EndEffector subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoDriveToHPAndPrep(ReefLocation start, DriveTrain driveTrain, Elevator elevator, CoralEffector coralEffector, 
          AllianceSelection alliance, TrajectoryCache cache, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoDriveToHPAndPrep", "Init", log, "startingReefLocation", start.toString()),
      new ParallelCommandGroup(
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, AutoSelection.getReefToHP(start), driveTrain, alliance, log)
        //new ElevatorSetPosition(ElevatorPosition.CORAL_HP.value, elevator, endEffector, log)    
      )
    );
  }
}
