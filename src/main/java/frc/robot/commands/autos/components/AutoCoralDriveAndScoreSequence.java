// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.DataLogMessage;
import frc.robot.commands.sequences.AutomatedDriveToReefAndScoreCoral;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoCoralDriveAndScoreSequence extends SequentialCommandGroup {
  /**
   * Drive from barge to end (reef location) and score a coral.
   * @param fromHP true = starting at HP, false = starting at barge
   * @param score true = we want to move the elevator to score the coral false = stop at reef DOESN'T WORK
   * @param location ReefLocation (A-L) to end at
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param hopper Hopper subsystem
   * @param rightJoystick Joystick joystick
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection alliance
   * @param field Field field
   */
  public AutoCoralDriveAndScoreSequence(boolean fromHP, boolean score, ReefLocation location, ReefLevel level, DriveTrain driveTrain,
      Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, Joystick rightJoystick, AllianceSelection alliance,
      Field field) {
    addCommands(
      new DataLogMessage(false, "AutoCoralDriveAndScoreSequence: Start, goal reef location =", location.toString()),
      // Drive to reef while intaking to ensure coral is intaked (timeout on the intake command for 4 seconds)
      new AutoDriveToReef(fromHP, location, driveTrain, elevator, wrist, coralEffector, hopper, alliance),
      
      // Wait for up to 2 seconds for coral to be fully intaked. If coral isn't in by then, go back to loading station
      new WaitUntilCommand(() -> coralEffector.isCoralPresent()).withTimeout(2),

      // If coral is detected, score; if not, then end the score sequence and skip to the next part of the auto
      either(
        new AutomatedDriveToReefAndScoreCoral(location, score, level, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, field),
        none(),
        () -> coralEffector.isCoralPresent()
      ),


      new DataLogMessage(false, "AutoCoralDriveAndScoreSequence: End")
    );
  }
}
