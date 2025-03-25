// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;


/**
 * Intakes algae by moving the wrist and elevator to the indicated intake position, and then 
 * running the algaeGrabber until the algae is in the mechanism. Then backs up the robot.
 * @param position position to move elevator to (use ElevatorWristConstants.ElevatorWristPosition.ALGAE_...)
 * @param driveTrain DriveTrain subsystem
 * @param elevator Elevator subsystem
 * @param wrist Wrist subsystem
 * @param algaeGrabber AlgaeGrabber subsystem
 * @param log FileLog utility
 */
public class AlgaeIntakeSequence extends SequentialCommandGroup {
  public AlgaeIntakeSequence(ElevatorWristPosition position, DriveTrain driveTrain, Elevator elevator, Wrist wrist, AlgaeGrabber algaeGrabber) {
    addCommands(
      new WristElevatorSafeMove(position, RegionType.STANDARD, elevator, wrist),
      new AlgaeGrabberIntake(algaeGrabber)
      // either(
      //   new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
      //       0.5, 1.0,
      //       TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees,
      //       true, true, driveTrain).asProxy(),
      //   none(),
      //   () -> position == ElevatorWristPosition.ALGAE_LOWER || position == ElevatorWristPosition.ALGAE_UPPER
      // )
    );
  }
}
