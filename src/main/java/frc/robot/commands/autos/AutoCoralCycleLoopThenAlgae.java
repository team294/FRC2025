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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.commands.sequences.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;

public class AutoCoralCycleLoopThenAlgae extends SequentialCommandGroup {
  /**
   * Currently written for center auto only. Scores GH algae and then IJ algae into barge.
   * Scores each coral in the locations of the given list at the given level, then grabs the algae at the ending position and backs up.
   * @param reefLocations list of ReefLocation to visit, in order
   * @param reefLevels list of ReefLevel to score on, in order
   * @param scoreFirstAlgae true = score the algae picked up, false = only pick up algae
   * @param grabSecondAlgae true = drive to IJ algae and pick it up (made for only center auto), false = end after scoring one. Can only be true if scoreFirstAlgae is true
   * @param scoreSecondAlgae true = move elevator up, score algae from behind the start line, move elevator down, false = skip the aforementioned sequence. Can only be true if scoreFirstAlgae is true
   * @param driveTrain DriveTrain subsytem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param climber Climber subsystem
   * @param rightJoystick Joystick joystick
   * @param alliance AllianceSelection alliance
   * @param field Field field
   */
  public AutoCoralCycleLoopThenAlgae(List<ReefLocation> reefLocations, List<ReefLevel> reefLevels, boolean scoreFirstAlgae, boolean grabSecondAlgae, boolean scoreSecondAlgae, DriveTrain driveTrain, Elevator elevator, 
      Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, Climber climber, Joystick rightJoystick, AllianceSelection alliance, Field field, TrajectoryCache cache) {
    
    addCommands(
      new DataLogMessage(false, "AutoCoralCycleLoopThenAlgae: Start")
    );

    // No reef locations provided, so do nothing
    if (reefLocations == null || reefLocations.size() == 0) {
      addCommands(
        new ClimberSetRatchet(false, climber)
      );
    }

    else {
      //int lastIndex = reefLocations.size() - 1;
      //ReefLocation lastCoralLocation = reefLocations.get(lastIndex);
      //ElevatorWristPosition position = lastCoralLocation.isAlgaeLower ? ElevatorWristPosition.ALGAE_LOWER : ElevatorWristPosition.ALGAE_UPPER;

      addCommands(
        // First, do the loop for the coral cycles (ends at reef, bumpers distanceFromReefToScore away from reef aka 6.25 inches)
        new AutoCoralCycleLoop(reefLocations, reefLevels, true, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, alliance, field),
        
        // Drive and intake algae from the reef (from GH only)
        new AutomatedDriveToReefAndIntakeAlgae(AlgaeLocation.GH, driveTrain, elevator, wrist, algaeGrabber, field).until(() -> algaeGrabber.isAlgaePresent()),
        
      
        // If we want to score algae, then score it. If not, just back up and end auto
        either(
          sequence(
            deadline(
              // Drive to barge while moving elevator up until reached barge position
              new DriveToBargeWithOdometry(driveTrain, field),
              sequence(
                new WaitCommand(1),
                new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_LOWER, RegionType.STANDARD, elevator, wrist)
              )
            ),
            // Scoring
            new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_NET, RegionType.STANDARD, elevator, wrist),
            new AlgaeGrabberOuttake(algaeGrabber).withTimeout(0.5), 
            // Start lowering elevator, then start moving
            new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_LOWER, RegionType.STANDARD, elevator, wrist).until(
              () -> elevator.getElevatorPosition() < ElevatorWristPosition.CORAL_L3.elevatorPosition),
          
            // Now, we go to grab a second algae, IJ, if the boolean to do so is true
            either(
              sequence(
                // Drive partially to IJ with trajectory, then finish driving and grab algae (we can figure out avoiding a stop if necessary after verifying that this works)
                deadline(
                  new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(TrajectoryName.BargeScoringToIJ), driveTrain, alliance).withTimeout(cache.getTrajectory(TrajectoryName.BargeScoringToIJ).getTotalTime() - 0.3),
                  new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_LOWER, RegionType.STANDARD, elevator, wrist)
                ),
                new AutomatedDriveToReefAndIntakeAlgae(AlgaeLocation.IJ, driveTrain, elevator, wrist, algaeGrabber, field).until(() -> algaeGrabber.isAlgaePresent()),
                
                // Drive towards barge from IJ position and stop before start line
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(TrajectoryName.EndCenterAuto), driveTrain, alliance),


                // Scoring, if the boolean to do so is true
                either(
                  sequence(
                    new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_NET, RegionType.STANDARD, elevator, wrist),
                    new AlgaeGrabberSetPercent(-0.5, algaeGrabber), // TODO verify that this outtake speed is ok, 9/26/25 Chezy updated from -.6 to -.62
                    new WaitCommand(0.4), // Changing back to wait time because the algae might not be completely out of the grabber the second the bump switch is no longer activated
                    new AlgaeGrabberStop(algaeGrabber),
                    new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.STANDARD, elevator, wrist)
                  ),
                  none(),
                  () -> scoreSecondAlgae
                )

                // Drive to barge, move elevator up, score, move elevator down.
                // deadline(
                //   // Drive to barge while moving elevator up until reached barge position
                //   new DriveToBargeWithOdometry(driveTrain, field),
                //   sequence(
                //     new WaitCommand(1),
                //     new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_LOWER, RegionType.STANDARD, elevator, wrist)
                //   )
                // ),    
                // new WristElevatorSafeMove(ElevatorWristPosition.ALGAE_NET, RegionType.STANDARD, elevator, wrist),
                // new AlgaeGrabberOuttake(algaeGrabber).withTimeout(0.5), 
                // new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist)
              ), 
              none(), 
              () -> grabSecondAlgae
            )
          ),

          new DriveToPose(CoordType.kRelative, new Pose2d(-0.3, 0, Rotation2d.kZero), driveTrain),
          () -> scoreFirstAlgae
        ),
  
        new DataLogMessage(false, "AutoCoralCycleLoopThenAlgae", "End")
      );
    }
  }
}
