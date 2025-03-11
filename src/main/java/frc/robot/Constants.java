// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.TrapezoidProfileBCR;

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
    public static final String CANDrivetrainBus = "CANivore";

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
    public static final int CANClimberEncoder = 14;

    public static final int CANElevator1 = 15;  // Right
    public static final int CANElevator2 = 16;  // Left
    public static final int CANHopper = 17;
    public static final int CANCoralEffector = 18;
    public static final int CANAlgaeGrabber = 19;
    public static final int CANWrist = 20;
    public static final int CANClimber = 21;

    public static final int CANPigeonGyro = 22;

    public static final int CANdle = 23;

    // Digital IO Ports
    public static final int DIOElevatorLowerLimitSensor1 = 1; // Right
    public static final int DIOElevatorLowerLimitSensor2 = 0; // Left
    public static final int DIOAlgaeGrabberBumpSwitch = 2;
    public static final int DIOCoralEffectorExitSensor = 3;
    public static final int DIOCoralEffectorEntrySensor = 4;
  }

  public static final class OIConstants {
    public static final int usbXboxController = 0;
    public static final int usbLeftJoystick = 1;
    public static final int usbRightJoystick = 2;
    public static final int usbCoPanel = 3;

    public static final double joystickDeadband = 0.01;
    public static final double joystickJoggingDeadband = 10.0 * joystickDeadband;
    public static final double controllerDeadband = 0.05;
  }
  
  // TODO CALIBRATE FOR 2025
  public static final class RobotDimensions {
    // Drivebase adjustment for path-of-wheel diameter when turning in place
    private static final double DrivetrainAdjustmentFactor = 0.9911;      // TODO CALIBRATE

    // Left-right distance between the drivetrain wheels, measured from center to center, in meters
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.75) * DrivetrainAdjustmentFactor;

    // Front-back distance between the drivetrain wheels, measured from center to center, in meters
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.75) * DrivetrainAdjustmentFactor;
    
    // Width of robot in meters plus bumpers, in meters
    public static final double robotWidth = Units.inchesToMeters(36.5);   // TODO CALIBRATE (for DriveToReef)

    // Diagonal width of robot, in meters
    public static final double robotDiagonal = Math.sqrt(2) * robotWidth;
  }

  public static final class SwerveConstants {
    // Past kDriveGearRatio: L3 = 6.12:1. Mk4i = 8.14:1 (L1-std gears), 6.75:1 (L2-fast gears), 5.903 (modified L2 16-tooth gear).
    // Past kTurningGearRatio: Mk4i = 150 / 7:1.

    // Encoder calibration to meters traveled or wheel facing degrees
    public static final double kEncoderCPR = 1.0;                                              // CALIBRATED Encoder counts per revolution of motor pinion gear
    public static final double kDriveGearRatio = (5.90 / 1.0);                                 // CALIBRATED Mk4n = 5.90:1 (L2+)
    public static final double kTurningGearRatio = (18.75 / 1.0);                              // CALIBRATED Mk4n = 18.75:1
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4) * 0.9683;        // CALIBRATED Wheels are nominal 4"
    public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
    public static final double kTurningEncoderDegreesPerTick = 360.0 / kEncoderCPR / kTurningGearRatio;
  
    // Robot calibration for feed-forward and max speeds
    public static final double voltageCompSaturation = 12.0;

    // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
    // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
    // and further derate (initial test by 5%) to account for some battery droop under heavy loads.

    public static final double kMaxSpeedMetersPerSecond = 5.1;  // TODO CALIBRATE FOR 2025
    public static final double kFullSpeedMetersPerSecond = 0.95 * kMaxSpeedMetersPerSecond;
    public static final double kNominalSpeedMetersPerSecond = 0.5 * kMaxSpeedMetersPerSecond;

    public static final double kFineControlMaxSpeedMetersPerSecond = 1;
    public static final double kFineControlMaxTurningRadiansPerSecond = 1;

    // TODO CALIBRATE FOR 2025
    public static final double kMaxAccelerationMetersPerSecondSquare = 7.5;
    public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
    public static final double kNominalAccelerationMetersPerSecondSquare = 3.5;
    public static final double kMaxTurningRadiansPerSecond = 11.0;
    public static final double kNominalTurningRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;     // Not used in code currently
    public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;

    // TODO CALIBRATE FOR 2025
    public static final double kVDriveAvg = 2.350;  // In voltage per meters/second
    private static final double kVmFLrel = 1.0;     // kV modifier for FL drive motor
    private static final double kVmFRrel = 1.0;     // kV modifier for FR drive motor
    private static final double kVmBLrel = 1.0;     // kV modifier for BL drive motor
    private static final double kVmBRrel = 1.0;     // kV modifier for BR drive motor

    // Normalize kVm constants
    private static double kVmAvg = (kVmFLrel + kVmFRrel + kVmBLrel + kVmBRrel) / 4.0;
    public static final double kVmFL = kVmFLrel / kVmAvg;
    public static final double kVmFR = kVmFRrel / kVmAvg;
    public static final double kVmBL = kVmBLrel / kVmAvg;
    public static final double kVmBR = kVmBRrel / kVmAvg;

    public static final double dt = 0.02;             // CALIBRATED Timestep for discretizing robot motion, in seconds (scheduler time period = 20ms)
    public static final double kADrive = 0.1203;      // In voltage per meters per second^2 TODO CALIBRATE FOR 2025
    public static final double kADriveToPose = 0.050; // CALIBRATED on ETU in 3/2/2025 In seconds (On the ETU, DriveToPose behaves better with a small value for kADriveToPose)
    public static final double kSDrive = 0.002;       // In voltage TODO CALIBRATE FOR 2025

    // Minimum abs delta (in m/sec) between actual wheel velocity and desired wheel velocity for kADrive to be applied.
    // If the delta is less than this, then don't use kADrive. This prevents the drive motors from jittering.
    // Note that the swerve module kP in the velocity controller will still work to maintain the proper wheel speed.
    public static final double velMinDeltaUsingkA = 0.3;
  }
  public static final class DriveConstants {
    // The locations of the wheels relative to the physical center of the robot, in meters.
    // X: positive = forward. Y: positive = to the left
    // The order in which you pass in the wheel locations is the same order that you will receive 
    // the module states when performing inverse kinematics. It is also expected that
    // you pass in the module states in the same order when calling the forward kinematics methods.
    // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
      new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
      new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
      new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2)
    );

    // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
    // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
    // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
    public static double offsetAngleFrontLeftMotor = -56.6;
    public static double offsetAngleFrontRightMotor = -152.8;
    public static double offsetAngleBackLeftMotor = -56.9;
    public static double offsetAngleBackRightMotor = -8.3;

    // Theta kp value for joystick in rad/sec
    public static final double kPJoystickThetaController = 3;    
  }

  public static class FieldConstants {
    public static final double length = 17.548;  // CALIBRATED 57 ft 6 7/8 in = 1754.8 cm
    public static final double width = 8.052;    // CALIBRATED 26 ft 5 in = 805.2 cm

    public static enum ReefLevel {
      L1, L2, L3, L4
    }

    public static enum ReefLocation {
      A, B, C, D, E, F, G, H, I, J, K, L
    }

    // Calculated by measuring y distance between center of reef wall and reef pole (6.469731 in), converted to meters
    public static final double ReefScoringPositionAprilTagOffset = 0.164331496063;
  }

  public static class VisionConstants {
    public static class PhotonVisionConstants {        
      public static final Transform3d robotToCamLeft = new Transform3d(
        new Translation3d(Units.inchesToMeters(7.25), Units.inchesToMeters(13.75), Units.inchesToMeters(14.5)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(-28)) // Mounted facing forward-right on left side of robot
      );
      public static final Transform3d robotToCamRight = new Transform3d(
        new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(-14), Units.inchesToMeters(15)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(26))  // Mounted facing forward-left on right side of robot
      );

      public static final String leftAprilTagCameraName = "AprilTagCameraLeft";
      public static final String rightAprilTagCameraName = "AprilTagCameraRight";
    }
  }

  public static final class TrajectoryConstants {
    // Max error for robot rotation
    public static final double maxThetaErrorDegrees = 1.0;
    public static final double maxPositionErrorMeters = 0.04; // 1.6 inches

    // Max error for interim positions (not final)
    public static final double interimThetaErrorDegrees = 2.0;        
    public static final double interimPositionErrorMeters = 0.20; // 8 inches

    public static final double xError = 0.25;
    public static final double yError = 0.25;
    public static final double choreoMaxThetaErrorDegrees = 5;
    public static final double endVelocityErrorDegrees = 0.75;

    // Feedback terms for holonomic drive controllers

    // X-velocity controller kP. Units = (meters/sec of velocity) / (meters of position error)
    public static final double kPXController = 4;

    // Y-velocity controller kP. Units = (meters/sec of velocity) / (meters of position error)  
    public static final double kPYController = 4; 

    // Theta-velocity controller kP. Units = (rad/sec of velocity) / (radians of angle error)
    public static final double kPThetaController = 3;

    public static final TrajectoryConfig swerveTrajectoryConfig = new TrajectoryConfig(
        SwerveConstants.kNominalSpeedMetersPerSecond,
        SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
        .setKinematics(DriveConstants.kDriveKinematics);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        SwerveConstants.kNominalTurningRadiansPerSecond,
        SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

    // Constraint for the DriveToPose motion profile for distance being travelled
    public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints = new TrapezoidProfileBCR.Constraints(
        SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
  }

  public static final class HopperConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.1;         // TODO CALIBRATE FOR 2025
    public static final double reverseIntakePercent = -0.1; // TODO CALIBRATE FOR 2025
  }

  public static final class CoralEffectorConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.1;   // CALIBRATED
    public static final double outtakePercent = 0.4;  // CALIBRATED
  }

  public static final class AlgaeGrabberConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.4; // CALIBRATED
    public static final double outtakePercent = -1; // CALIBRATED
  }

  public static final class ElevatorConstants {
    public static final double kEncoderCPR = 1.0;                 // Encoder counts per revolution of the motor pinion gear
    public static final double kElevGearRatio = (5.0 / 1.0);      // Gear reduction ratio between Kraken and gear driving the elevator CALIBRATED FOR 2025 (5:1)
    public static final double kElevPulleyDiameterInches = 1.504; // Diameter of the pulley driving the elevator in inches TODO CALIBRATE FOR 2025 (1.504" nominal)
    public static final double kElevEncoderInchesPerTick = (kElevPulleyDiameterInches * Math.PI) / kEncoderCPR / kElevGearRatio;
    
    public static final double compensationVoltage = 12.0;

    public static final double maxUncalibratedPercentOutput = 0.05;
    public static final double maxManualPercentOutput = 0.1;  // Max elevator speed when driven using Xbox controller
    public static final double maxPercentOutput = 1.0;        // Absolute max output to elevator motors

    // TODO CALIBRATE FOR 2025
    public enum ElevatorPosition {
      LOWER_LIMIT(0.0),
      UPPER_LIMIT(82.0);
 
      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final double value;
      ElevatorPosition(double value) { this.value = value; }
    }
  }

  public static final class WristConstants {
    // Gear Ratio (convention from CTRE library) = the ratio of motor rotor rotations to wrist rotations,
    // where a ratio greater than 1 is a reduction.
    public static final double kWristGearRatio = ((60.0 / 8.0) * (48.0 / 16.0));          // CALIBRATED FOR 2025 (22.5:1)
    public static final double kWristDegreesPerRotation = 360.0;                          // Wrist degrees per rotation of the cancoder
    
    public static final double compensationVoltage = 12.0;
    public static final double maxUncalibratedPercentOutput = 0.1;  // TODO CALIBRATE FOR 2025
    public static final double maxManualPercentOutput = 0.05;       // TODO CALIBRATE FOR 2025
    public static final double maxPercentOutput = 0.2;              // TODO CALIBRATE FOR 2025

    // Should be updated in RobotPreferences, so it cannot be final
    public static double offsetAngleCANcoder = 0.0;                 // CANCoder raw angle (in degrees) when arm is at 0 degrees.  TODO CALIBRATE FOR 2025

    // 1 makes absolute position unsigned [0, 1); 0.5 makes it signed [-0.5, 0.5), 0 makes it always negative
    public static double cancoderDiscontinuityPoint = 1.0;          // TODO CALIBRATE FOR 2025 - should be the center of the region of unallowed motion

    public static final double kP = 0.0;    // TODO CALIBRATE FOR 2025    kP = (desired-output-volts) / (error-in-wrist-rotations)
    public static final double kI = 0.0;    // TODO CALIBRATE FOR 2025
    public static final double kD = 0.0;    // TODO CALIBRATE FOR 2025
    public static final double kG = 0.0;  // TODO CALIBRATE FOR 2025    kG = Feed foward voltage to add to hold wrist horizontal (0 deg)
    public static final double kS = 0.0; // TODO CALIBRATE FOR 2025
    public static final double kV = 0.0; // TODO CALIBRATE FOR 2025

    public static final double MMCruiseVelocity = 0.25;                   // Max velocity in wrist rotations / second TODO CALIBRATE FOR 2025
    public static final double MMAcceleration = MMCruiseVelocity / 0.35;  // Max acceleration in wrist rotations / second^2. MMVel / MMAccel = seconds to full velocity. TODO CALIBRATE FOR 2025
    public static final double MMJerk = MMAcceleration / 0.05;            // Max jerk in wrist rotations / second^3. MMAccel / MMJerk = seconds to full acceleration. TODO CALIBRATE FOR 2025

    // TODO add wrist regions
    public enum WristRegion {
      main,
      uncalibrated;
    }

    // TODO CALIBRATE FOR 2025
    public enum WristAngle {
      LOWER_LIMIT(-30.0),
      UPPER_LIMIT(90.0);

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final double value;
      WristAngle(double value) { this.value = value; }
    }
  }

  public static final class ElevatorWristConstants {
    public enum ElevatorWristPosition {
      START_CONFIG(0.0, 90.0),

      CORAL_HP(0.0, 69.0),

      CORAL_L1(25.56, 55.0),
      CORAL_L2(25.56, 55.0),
      CORAL_L3(40.78, 55.0),
      CORAL_L4(70.7, 20.0),

      ALGAE_GROUND(5.8, -18.5),
      ALGAE_LOWER(34.0, -15.0),
      ALGAE_UPPER(49.7, -15.0),

      ALGAE_PROCESSOR(9.84, 0.0),
      ALGAE_NET(63.0, 70.0);

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final double elevatorPosition;
      public final double wristAngle;

      ElevatorWristPosition(double elevatorPosition, double wristAngle) {
        this.elevatorPosition = elevatorPosition;
        this.wristAngle = wristAngle;
      }
    }
  }

  public static final class ClimberConstants {
    // Gear Ratio (convention from CTRE library) = the ratio of motor rotor rotations to wrist rotations,
    // where a ratio greater than 1 is a reduction.
    public static final double kClimberGearRatio = (135.0/1.0);          // CALIBRATED FOR 2025  (135:1)
    public static final double kClimberDegreesPerRotation = 360.0;                          // Wrist degrees per rotation of the cancoder
    
    public static final double compensationVoltage = 12.0;
    public static final double maxUncalibratedPercentOutput = 0.1;  // CALIBRATED
    public static final double maxManualPercentOutput = 0.2;        // CALIBRATED
    public static final double maxPercentOutput = 0.2;              // CALIBRATED

    // Should be updated in RobotPreferences, so it cannot be final
    public static double offsetAngleCANcoder = 37.7;                 // CANCoder raw angle (in degrees) when arm is at 0 degrees.  CALIBRATED
    // 1 makes absolute position unsigned [0, 1); 0.5 makes it signed [-0.5, 0.5), 0 makes it always negative
    // This value is the center of the region of *unallowed* motion
    public static double cancoderDiscontinuityPoint = 0.74;          // CALIBRATED FOR 2025


    public static final double kP = 0.0;    // TODO CALIBRATE FOR 2025      kP = (desired-output-volts) / (error-in-wrist-rotations)
    public static final double kI = 0.0;    // CALIBRATED
    public static final double kD = 0.0;    // CALIBRATED
    public static final double kG = 0.0;    // CALIBRATED      kG = Feed foward voltage to add to hold wrist horizontal (0 deg)
    public static final double kS = 0.096;   // CALIBRATED   kS = (volts)
    public static final double kV = 15.2;    // CALIBRATED   kV = (volts)/(wrist-rotations/sec)

    public static final double MMCruiseVelocity = 50.0/360.0;             // Max velocity in climber rotations / second
    public static final double MMAcceleration = MMCruiseVelocity / 0.35;  // Max acceleration in climber rotations / second^2. MMVel / MMAccel = seconds to full velocity.
    public static final double MMJerk = MMAcceleration / 0.05;            // Max jerk in climber rotations / second^3. MMAccel / MMJerk = seconds to full acceleration.

    // CALIBRATED
    public enum ClimberAngle {
      LOWER_LIMIT(-82.0),
      UPPER_LIMIT(176.0),
      CALIBRATE_MANUAL(-84.0),

      CLIMB_START(77.0),
      CLIMB_END(176.0);

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final double value;
      ClimberAngle(double value) { this.value = value; }
    }
  }
}
