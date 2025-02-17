// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utilities.TrapezoidProfileBCR;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public enum CoordType {
    kRelative,              // Relative to current robot location/facing
    kAbsolute,              // Absolute field coordinates, don't reset robot pose
    kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
    kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
  }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
      kNoStop,
      kCoast,
      kBrake;
  }

  public static final class Ports{
    // public static final int CANPneumaticHub = 1;

    public static final String CANDrivetrainBus = "rio";    // Specify which CANbus the drivetrain is on: "rio" or "CANivore"

    public static final int CANDriveFrontLeftMotor = 1;
    public static final int CANDriveFrontRightMotor = 2;
    public static final int CANDriveBackLeftMotor = 3;
    public static final int CANDriveBackRightMotor = 4;

    public static final int CANDriveTurnFrontLeftMotor = 5;
    public static final int CANDriveTurnFrontRightMotor = 6;
    public static final int CANDriveTurnBackLeftMotor = 7;
    public static final int CANDriveTurnBackRightMotor = 8;

    // Note:  Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
    // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
    // This applies to the turn encoders, which are used as remote sensors for the turn motors.
    public static final int CANTurnEncoderFrontLeft = 9;
    public static final int CANTurnEncoderFrontRight = 10;
    public static final int CANTurnEncoderBackLeft = 11;
    public static final int CANTurnEncoderBackRight = 12;

    public static final int CANEndEffector = 13;

    public static final int CANElevator1 = 14;
    public static final int CANElevator2 = 15;

    public static final int CANPigeonGyro = 18;
    public static final int CANPigeonGyro2 = 19;

    public static final int CANdle = 21;

    public static final int CANHopper = 23;

    // Digital IO Ports
    public static final int DIOElevatorLowerLimitSensor = 3;
    public static final int DIOElevatorUpperLimitSensor = 4;
    public static final int DIOEndEffectorEntrySensor = 0;
    public static final int DIOEndEffectorExitSensor = 1;
  }

  public static final class OIConstants {
    //Ports from last year
    public static final int usbXboxController = 0;
    public static final int usbLeftJoystick = 1;
    public static final int usbRightJoystick = 2;
    public static final int usbCoPanel = 3;

    public static final double joystickDeadband = 0.01;
    public static final double controllerDeadband = 0.05;
  }
  
  public static final class RobotDimensions {
    // Drivebase adjustment for path-of-wheel diameter when turning in place
    private static final double DrivetrainAdjustmentFactor = 0.9911;     // CALIBRATED on 1/15/2025
    // Left-right distance between the drivetrain wheels; should be measured from center to center
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24.25) * DrivetrainAdjustmentFactor;     // CALIBRATED.  ETU bot CAD = 24.25".
    // Front-back distance between the drivetrain wheels; should be measured from center to center
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.25) * DrivetrainAdjustmentFactor;      // CALIBRATED.  ETU bot CAD = 24.25".
    
    // Width of robot in meters, + bumpers
    public static final double robotWidth = Units.inchesToMeters(36.5); //Was 0.9144 (meters)
    // Diagonal Width of robot in meters
    public static final double robotDiagonal = Math.sqrt(2)*robotWidth;
  }

  public static class FieldConstants {
    public static final double length = 17.55;      // 57 ft 6 7/8 in = 1755 cm
    public static final double width = 8.05;        // 26 ft 5 in = 805 cm
  }

  public static final class HopperConstants {
    public static final double compensationVoltage = 12.0;
    public static final double hopperIntakePercent = 0.16; // TODO CALIBRATE FOR 2025
    public static final double hopperReverseIntakePercent = -0.16; // TODO CALIBRATE FOR 2025
  }

}