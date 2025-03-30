// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutoCoralCycleLoopThenAlgae extends SequentialCommandGroup {
  /**
   * Scores each coral in the locations of the given list at the given level, then grabs the algae at the ending position and backs up.
   * @param reefLocations list of ReefLocation to visit, in order
   * @param reefLevels list of ReefLevel to score on, in order
   * @param reefLevel ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsytem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param led LED subsystem
   * @param alliance AllianceSelection alliance
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoCoralCycleLoopThenAlgae(List<ReefLocation> reefLocations, List<ReefLevel> reefLevels, DriveTrain driveTrain, Elevator elevator, 
      Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, LED led, AllianceSelection alliance, TrajectoryCache cache) {
    
    // No reef locations provided, so do nothing
    if (reefLocations == null || reefLocations.size() == 0) {
      addCommands(none());
    }

    else {
      int lastIndex = reefLocations.size() - 1;
      ReefLocation lastCoralLocation = reefLocations.get(lastIndex);
      ElevatorWristPosition position = lastCoralLocation.isAlgaeLower ? ElevatorWristPosition.ALGAE_LOWER : ElevatorWristPosition.ALGAE_UPPER;
      double yRelativeOffset = lastCoralLocation.onRightSide ? 0.2 : -0.2; 

      addCommands(
        new DataLogMessage(false, "AutoCoralCycleLoopThenAlgae", "Start", lastCoralLocation, lastCoralLocation.toString(), "onRightSide", lastCoralLocation.onRightSide, "yRelativeOffset", yRelativeOffset),

        // First, do the loop for the coral cycles (ends at reef)
        new AutoCoralCycleLoop(reefLocations, reefLevels, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, led, alliance, cache),
        
        // Grab algae from current position (back up, prep wrist and elevator, and then go forward and intake) and back up

        // Back up based on an offset (if it is the right or left branch) TODO calibrate distance
        new DriveToPose(CoordType.kRelative, new Pose2d(-0.65, yRelativeOffset, new Rotation2d(0)), driveTrain),
        
        // Prep the wrist and elevator for intaking from the reef
        new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist),

        // Start running the algaeGrabber and drive into the reef TODO calibrate distance
        parallel(
          // Cannot use AlgaeIntakeSequence due to the commands in parallel both requiring the driveTrain
          // new AlgaeIntakeSequence(position, driveTrain, elevator, wrist, algaeGrabber),
          sequence(
            new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist),
            new AlgaeGrabberIntake(algaeGrabber)
          ),
          new DriveToPose(CoordType.kRelative, new Pose2d(0.6, 0, new Rotation2d(0)), driveTrain)
        ),

        // After intaking the algae, back up TODO calibrate distance
        new DriveToPose(CoordType.kRelative, new Pose2d(-0.5, 0, new Rotation2d(0)), driveTrain),

        new DataLogMessage(false, "AutoCoralCycleLoopThenAlgae", "End")
      );
    }
  }
}
