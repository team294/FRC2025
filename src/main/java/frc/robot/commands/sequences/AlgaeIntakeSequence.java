// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.LEDEventManager;


/**
 * Intakes algae by moving the wrist and elevator to the indicated intake position, and then 
 * running the algaeGrabber until the algae is in the mechanism. Then backs up the robot.
 * @param position position to move elevator to (use ElevatorWristConstants.ElevatorWristPosition.ALGAE_...)
 * @param driveTrain DriveTrain subsystem
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param ledEventManager LEDEventManager utility
 */
public class AlgaeIntakeSequence extends SequentialCommandGroup {
  public AlgaeIntakeSequence(ElevatorWristPosition position, DriveTrain driveTrain, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber, LEDEventManager ledEventManager) {
    addCommands(
      
      deadline(
        parallel(
          new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist),
          new AlgaeGrabberIntake(algaeGrabber)
        ),
        runOnce(() -> ledEventManager.sendEvent(LEDEventManager.StripEvents.ALGAE_INTAKING))
      ),
      
      // new LEDSendNeutral(led),
      either(
        runOnce(() -> ledEventManager.sendEvent(LEDEventManager.StripEvents.ALGAE_MODE)),
        runOnce(() -> ledEventManager.sendEvent(LEDEventManager.StripEvents.NEUTRAL)), 
        () -> algaeGrabber.isAlgaePresent()
      ),

      either(
        sequence(
          new AlgaeGrabberSetPercent(0.1, algaeGrabber),
          waitSeconds(3),
          new AlgaeGrabberStop(algaeGrabber)
        ),
        none(),
        () -> position == ElevatorWristPosition.ALGAE_LOWER || position == ElevatorWristPosition.ALGAE_UPPER
      ),

      either(
        new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.STANDARD, elevator, wrist),
        none(),
        () -> position == ElevatorWristPosition.ALGAE_GROUND || position == ElevatorWristPosition.ALGAE_LOLLIPOP
      )
    );
  }
}
