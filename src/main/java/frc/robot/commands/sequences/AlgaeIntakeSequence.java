// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.LEDEventUtil;


/**
 * Intakes algae by moving the wrist and elevator to the indicated intake position, and then 
 * running the algaeGrabber until the algae is in the mechanism.
 * @param position position to move elevator to (use ElevatorWristConstants.ElevatorWristPosition.ALGAE_...)
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 */
public class AlgaeIntakeSequence extends SequentialCommandGroup {
  public AlgaeIntakeSequence(ElevatorWristPosition position, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber) {
    addCommands(
      new DataLogMessage(false, "AlgaeIntakeSequence: Start"),
      
      parallel(
        sequence(
          new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist),
          parallel(
            new AlgaeGrabberIntake(algaeGrabber),
            either(
              sequence(
                waitSeconds(0.5),
                new WristSetAngle(wrist.getWristAngle() + 5.0, wrist)
              ),
              none(),
              () -> position == ElevatorWristPosition.ALGAE_LOWER || position == ElevatorWristPosition.ALGAE_UPPER
            )
          )
        ),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ALGAE_INTAKING))
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      either(
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ALGAE_MODE)),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)), 
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
      ),

      new DataLogMessage(false, "AlgaeIntakeSequence: End")
    );
  }
}
