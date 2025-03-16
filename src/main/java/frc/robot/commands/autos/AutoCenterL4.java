// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.CoralEffectorOuttake;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.VisionOdometryStateSet;
import frc.robot.commands.WristElevatorSafeMove;
import frc.robot.commands.sequences.CoralScorePrepSequence;
import frc.robot.commands.sequences.DriveToReefWithOdometryForCoral;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;


public class AutoCenterL4 extends SequentialCommandGroup {
  public AutoCenterL4(DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Field field, Joystick rightJoystick, AllianceSelection allianceSelection, FileLog log) {
    addCommands(

      // Turn on vision odometry
      new VisionOdometryStateSet(true, driveTrain, log),

      // Reset pose so robot is facing the correct direction
      new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain, log),

      // Drive forward to get in april tag range
      new DriveToPose(CoordType.kRelative, new Pose2d(Units.inchesToMeters(20), 0, Rotation2d.kZero), driveTrain, log),

      // Drive to reef
      new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick, log),
      // new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick, log),

      // Move elevator to L4
      new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L4, elevator, wrist, algaeGrabber, log),

      // Drive forward to get to the reef (offset copied from DriveToReefWithOdometryForCoral, plus 4 inches)
      new DriveToPose(CoordType.kRelative, () -> new Pose2d((RobotDimensions.robotWidth / 2.0) + DriveConstants.driveBackFromReefDistance + Units.inchesToMeters(4), 0, new Rotation2d(0)), 
          0.5, 1.0, 
          TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
          true, true, driveTrain, log),

      // Wait for elevator to stabilize
      new WaitCommand(1.5),

      // Score coral on L4
      new CoralEffectorOuttake(coralEffector, log),

      // Wait after scoring
      new WaitCommand(1),

      // Back up for save elevator move
      new DriveToPose(CoordType.kRelative, () -> new Pose2d(-DriveConstants.driveBackFromReefDistance - 0.6, 0, new Rotation2d(0)), 
          0.5, 1.0, 
          TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
          true, true, driveTrain, log),

      // Move elevator back down
      new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist, log)
    );
  }
}
