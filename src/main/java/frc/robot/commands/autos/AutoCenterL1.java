// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.commands.*;
import frc.robot.commands.sequences.CoralScorePrepSequence;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutoCenterL1 extends SequentialCommandGroup {
  /**
   * Drives from the center of the start line to the front of the reef and scores the pre-loaded coral in L1.
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param allianceSelection AllianceSelection utility
   * @param log FileLog utility
   */
  public AutoCenterL1(DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, LED led, AllianceSelection allianceSelection) {
    addCommands(
      // Turn off vision odometry
      new VisionOdometryStateSet(false, driveTrain),

      // Reset pose so robot is facing the correct direction
      new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),

      parallel(
        // Drive to the reef using relative DriveToPose
        // The distance between the back of the start line and front of the reef is 89.95 inches (increased to prevent undershoot)
        new DriveToPose(CoordType.kRelative, new Pose2d(Units.inchesToMeters(91 - RobotDimensions.robotLength / 2.0), 0, new Rotation2d(0)), driveTrain),
        
        // Prep to score coral on L1
        new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L1, elevator, wrist, algaeGrabber)
      ),

      // Score coral on L1
      new CoralEffectorOuttake(coralEffector, led),

      // Wait for two seconds
      waitSeconds(2.0),

      // Back up from the reef
      new DriveToPose(CoordType.kRelative, new Pose2d(-Units.inchesToMeters(20), 0, new Rotation2d(0)), driveTrain),

      // Move the elevator and wrist to intake position
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist)
    );
  }
}
