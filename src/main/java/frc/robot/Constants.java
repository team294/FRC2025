// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public enum CoordType {
    kRelative,              // Relative to current robot location/facing
    kAbsolute,              // Absolute field coordinates, don't reset robot pose
    kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
    kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
  }

  // Options for driving stopping types
  public enum StopType {
    kNoStop,
    kCoast,
    kBrake;
  }

  public static final class Ports {
    // public static final int CANPneumaticHub = 1;

    // Specify which CANbus the drivetrain is on: "rio" or "CANivore"
    public static final String CANDrivetrainBus = "rio";

    public static final int CANDriveFrontLeftMotor = 1;
    public static final int CANDriveFrontRightMotor = 2;
    public static final int CANDriveBackLeftMotor = 3;
    public static final int CANDriveBackRightMotor = 4;

    public static final int CANDriveTurnFrontLeftMotor = 5;
    public static final int CANDriveTurnFrontRightMotor = 6;
    public static final int CANDriveTurnBackLeftMotor = 7;
    public static final int CANDriveTurnBackRightMotor = 8;

    // Note: Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
    // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
    // This applies to the turn encoders, which are used as remote sensors for the turn motors.
    public static final int CANTurnEncoderFrontLeft = 9;
    public static final int CANTurnEncoderFrontRight = 10;
    public static final int CANTurnEncoderBackLeft = 11;
    public static final int CANTurnEncoderBackRight = 12;

    public static final int CANWristEncoder = 13;

    public static final int CANHopper = 14;

    public static final int CANCoralEffector = 15;

    public static final int CANCoralGrabber = 16;

    public static final int CANElevator1 = 17;
    public static final int CANElevator2 = 18;

    public static final int CANPigeonGyro = 19;
    public static final int CANPigeonGyro2 = 20;

    public static final int CANWrist = 21;

    public static final int CANdle = 22;

    // Digital IO Ports
    public static final int DIOCoralEffectorEntrySensor = 0;
    public static final int DIOCoralEffectorExitSensor = 1;
    public static final int DIOAlgaeGrabberBumpSwitch1 = 2;
    public static final int DIOAlgaeGrabberBumpSwitch2 = 3;
  }

  public static final class OIConstants {
    public static final int usbXboxController = 0;
    public static final int usbLeftJoystick = 1;
    public static final int usbRightJoystick = 2;
    public static final int usbCoPanel = 3;

    public static final double joystickDeadband = 0.01;
    public static final double controllerDeadband = 0.05;
  }
  
  public static final class RobotDimensions {
    // Drivebase adjustment for path-of-wheel diameter when turning in place
    private static final double DrivetrainAdjustmentFactor = 0.9911;

    // Left-right distance between the drivetrain wheels, measured from center to center, in meters
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(24.25) * DrivetrainAdjustmentFactor;

    // Front-back distance between the drivetrain wheels, measured from center to center, in meters
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(24.25) * DrivetrainAdjustmentFactor;
    
    // Width of robot in meters plus bumpers, in meters
    public static final double robotWidth = Units.inchesToMeters(36.5);

    // Diagonal width of robot, in meters
    public static final double robotDiagonal = Math.sqrt(2) * robotWidth;
  }

  public static class FieldConstants {
    public static final double length = 17.55;  // 57 ft 6 7/8 in = 1755 cm
    public static final double width = 8.05;    // 26 ft 5 in = 805 cm
  }

  public static final class HopperConstants {
    public static final double compensationVoltage = 12.0;
    public static final double hopperIntakePercent = 0.1;         // TODO CALIBRATE FOR 2025
    public static final double hopperReverseIntakePercent = -0.1; // TODO CALIBRATE FOR 2025
  }

  public static class CoralEffectorConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.1;   // TODO CALIBRATE FOR 2025
    public static final double outtakePercent = -0.1; // TODO CALIBRATE FOR 2025
  }

  public static final class AlgaeGrabberConstants {
    public static final double compensationVoltage = 12.0;
    public static final double AlgaeGrabberIntakePercent = 0.1;   // TODO CALIBRATE FOR 2025
    public static final double AlgaeGrabberOuttakePercent = -0.1; // TODO CALIBRATE FOR 2025
  }

  public static final class WristConstants{
    public static final double kEncoderCPR = 1.0;                // TODO CALIBRATE FOR 2025
    public static final double kWristGearRatio = (5.0*5.0*3.0 * 48.0 / 22.0);   // TODO CALIBRATE FOR 2025
    public static final double kWristDegreesPerRotation =  360.0 / kEncoderCPR / kWristGearRatio; // TODO CALIBRATE FOR 2025

    
    public static final double voltageCompSaturation = 12.0;
    public static final double maxUncalibratedPercentOutput = 0.15;     // TODO CALIBRATE FOR 2025
    public static final double maxPercentOutput = 0.4;          // TODO CALIBRATE FOR 2025

    public static final double climbPercentOutput = -0.5; // TODO CALIBRATE FOR 2025

    // Should be updated in RobotPreferences, so it can't be final TODO: Define what position 0 should be
    public static double canCoderOffsetAngleWrist = 0.0; //TODO: CALIBRATE FOR 2025

    public static final double kP = 0.5;   // TODO CALIBRATE FOR 2025
    public static final double kI = 0.0; // TODO CALIBRATE FOR 2025
    public static final double kD = 0.0; // TODO CALIBRATE FOR 2025
    public static final double kG = 0.174;   // TODO CALIBRATE FOR 2025
    public static final double kS = 0.0367;  // TODO CALIBRATE FOR 2025
    public static final double kV = 0.1171;  // TODO CALIBRATE FOR 2025

    public static final double MMCruiseVelocity = 90.0;   // TODO CALIBRATE FOR 2025
    public static final double MMAcceleration = MMCruiseVelocity/0.35;    // TODO CALIBRATE FOR 2025
    public static final double MMJerk = MMAcceleration/0.05;  // TODO CALIBRATE FOR 2025

    public enum WristRegion {
      main,
      uncalibrated;
    }

    public enum WristAngle {
      lowerLimit(-83.0), // TODO CALIBRATE FOR 2025
      upperLimit(90.0); // TODO CALIBRATE FOR 2025

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final double value;
        WristAngle(double value) { this.value = value; }
    }
  }
}
