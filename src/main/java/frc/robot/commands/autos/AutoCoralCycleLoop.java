// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.*;
import frc.robot.commands.autos.components.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoCoralCycleLoop extends SequentialCommandGroup {
  /**
   * First, score the pre-loaded coral. Then, do cycles of (1) drive to HP, (2) intake, (3) drive to reef, and (4) score.
   * TODO Assumes each adjacent reef location in the list have a valid trajectory between them and HP
   * TODO Figure out proper exit conditions and timeouts
   * @param reefLocations list of ReefLocation to visit, in order
   * @param reefLevels list of ReefLevel to score on, in order
   * @param scoreLastCoral true = we want to move the elevator to score the coral false = stop at reef DOESN'T WORK
   * @param endAtHP true = end at the coral loading station, false = end at the reef 
   * @param driveTrain DriveTrain subsytem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param climber Climber subsystem
   * @param rightJoystick Joystick joystick
   * @param alliance AllianceSelection utility
   * @param field Field utility
   */
  public AutoCoralCycleLoop(List<ReefLocation> reefLocations, List<ReefLevel> reefLevels, boolean scoreLastCoral, boolean endAtHP, DriveTrain driveTrain, Elevator elevator, 
      Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, Climber climber, Joystick rightJoystick, AllianceSelection alliance, Field field) {

    addCommands(
      new DataLogMessage(false, "AutoCoralCycleLoop: Start")
    );

    // No reef locations provided, so do nothing
    if (reefLocations == null || reefLocations.size() == 0) {
      addCommands(
        new ClimberSetRatchet(false, climber)
      );
    }

    else {
      addCommands(
        new DriveResetPose(() -> AutoSelection.getBargeToReef(reefLocations.get(0)).getInitialPose(alliance.getAlliance() == Alliance.Red).get(), false, driveTrain),
        either(
          new InstantCommand(() -> coralEffector.setCoralEffectorPosition(coralEffector.getCoralEffectorPosition(), true)),
          none(),
          () -> coralEffector.isCoralPresent()
        )
      );

      // Score the pre-loaded coral with the first reef location
      if (reefLocations.size() >= 1 && reefLevels.size() >= 1) {
        addCommands(
          parallel(
            new AutoCoralDriveAndScoreSequence(false, false, reefLocations.get(0), reefLevels.get(0), driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, alliance, field),
            sequence(
              new ClimberSetRatchet(false, climber),
              new ClimberSetAngle(ClimberConstants.ClimberAngle.DEFAULT, climber)
            )
          )
        );
      }

      // After scoring the pre-load, do cycles of (1) drive to HP, (2) intake, (3) drive to reef, and (4) score
      if (reefLocations.size() >= 2 && reefLevels.size() >= 2) {
        // Generate AutoIntakeAndScoreCoralCycle for each pair of adjacent ReefLocations,
        // looping until the second-to-last element to ensure we do not go out of bounds
        for (int i = 0; i < reefLocations.size() - 1; i++) {
          if ((i + 1) >= reefLevels.size()) break;

          ReefLocation start = reefLocations.get(i);
          ReefLocation end = reefLocations.get(i + 1);
          boolean isLastCoral = !scoreLastCoral && reefLocations.get(i) == reefLocations.get(reefLocations.size() - 1); // TODO this boolean does not work

          addCommands(
            // If it is the final coral location, then isFinalCoral = true and the elevator does not move up. If not, then the elevator moves up to score
            new AutoCoralCycle(start, end, isLastCoral, reefLevels.get(i + 1), driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, alliance, field)
          );
        }
      }

      // Drive back to and end at HP after the last reef location if we are to end at HP. Otherwise, we are ending at reef 
      if (endAtHP && reefLocations.size() > 0) {
        addCommands(
          new AutoCoralDriveAndIntakeSequence(reefLocations.get(reefLocations.size() - 1), driveTrain, elevator, wrist, coralEffector, hopper, alliance)
        );
      }

      addCommands(new DataLogMessage(false, "AutoCoralCycleLoop: End"));
    }
  }
}