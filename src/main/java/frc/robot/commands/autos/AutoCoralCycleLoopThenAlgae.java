// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.commands.sequences.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutoCoralCycleLoopThenAlgae extends SequentialCommandGroup {
  /**
   * Scores each coral in the locations of the given list at the given level, then grabs the algae at the ending position and backs up.
   * @param reefLocations list of ReefLocation to visit, in order
   * @param reefLevels list of ReefLevel to score on, in order
   * @param driveTrain DriveTrain subsytem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param rightJoystick Joystick joystick
   * @param alliance AllianceSelection alliance
   * @param field Field field
   * @param log FileLog log
   */
  public AutoCoralCycleLoopThenAlgae(List<ReefLocation> reefLocations, List<ReefLevel> reefLevels, DriveTrain driveTrain, Elevator elevator, 
      Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, Joystick rightJoystick, AllianceSelection alliance, Field field) {
    
    // No reef locations provided, so do nothing
    if (reefLocations == null || reefLocations.size() == 0) {
      addCommands(none());
    }

    else {
      int lastIndex = reefLocations.size() - 1;
      ReefLocation lastCoralLocation = reefLocations.get(lastIndex);
      ElevatorWristPosition position = lastCoralLocation.isAlgaeLower ? ElevatorWristPosition.ALGAE_LOWER : ElevatorWristPosition.ALGAE_UPPER;
      double yRelativeOffset = lastCoralLocation.onRightSide ? 0.164 : -0.164; // From ReefScoringPositionAprilTagOffset  

      addCommands(
        new DataLogMessage(false, "AutoCoralCycleLoopThenAlgae", "Start", lastCoralLocation, lastCoralLocation.toString(), "onRightSide", lastCoralLocation.onRightSide, "yRelativeOffset", yRelativeOffset),

        // First, do the loop for the coral cycles (ends at reef, bumpers distanceFromReefToScore away from reef aka 6.25 inches)
        new AutoCoralCycleLoop(reefLocations, reefLevels, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, alliance, field),
      
        // Back up based on an offset (if it is the right or left branch) TODO calibrate distance, andrew estimates we should be ~7 inches away bumper to reef distance
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
