// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoPushFriendThenCoralCycle extends SequentialCommandGroup {
  /**
   * <b> ONLY STARTING ON LEFT SIDE OF THE REEF </b> (reef positions H, I, J, K, L, A) 
   * <p> Pushes friend :) into our alliance barge to secure auto RP, then goes back to starting location and does AutoCoralCycleLoop as normal
   * @param reefLocations list of ReefLocation to visit, in order
   * @param reefLevels list of ReefLevel to score on, in order
   * @param reefLevel ReefLevel (L1, L2, L3, L4) to score on
   * @param endAtHP true = end at the coral loading station, false = end at the reef 
   * @param grabAlgae true = grab algae at the end of coral cycle loop <b>(currently cannot end at HP)</b>, false = do not grab algae 
   * @param driveTrain DriveTrain subsytem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsysteem
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection utility
   * @param cache TrajectoryCache utility
   * @param log FileLog utility
   */
  public AutoPushFriendThenCoralCycle(List<ReefLocation> reefLocations, List<ReefLevel> reefLevels, boolean endAtHP, boolean grabAlgae, DriveTrain driveTrain, 
      Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, AllianceSelection alliance, TrajectoryCache cache, Field field, FileLog log) {    
    // Different starting pose based on if we are blue or red alliance
    Pose2d startingPose = AutoSelection.getBargeToReef(reefLocations.get(0)).getInitialPose(alliance.getAlliance() == Alliance.Red).get();
    
    addCommands(
      new DriveResetPose(startingPose, true, driveTrain, log),
            
      // Push friend! :D
      new DriveToPose(CoordType.kRelative, new Pose2d(-1.5, 0, new Rotation2d(0)), driveTrain, log).withTimeout(3),

      // Drive back to starting position
      new DriveToPose(CoordType.kAbsolute, startingPose, driveTrain, log),

      // Based on whether grabAlgae is true or not, do coral cycle loop with or without ending by grabbing coral
      either(
        new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, alliance, cache, field, log),
        new AutoCoralCycleLoop(reefLocations, reefLevels, endAtHP, driveTrain, elevator, wrist,coralEffector, algaeGrabber, hopper, alliance, cache, field, log),
        () -> grabAlgae
      )
    );
  }
}
