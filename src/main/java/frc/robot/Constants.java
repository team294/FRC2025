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


  public static final class SwerveConstants {
    // Encoder calibration to meters travelled or wheel facing degrees
  public static final double kEncoderCPR = 1.0;                           // CALIBRATED = 1.0.  Encoder counts per revolution of motor pinion gear.
  public static final double kDriveGearRatio = (6.12 / 1.0);              // CALIBRATED = L3 = 6.12:1.  Mk4i = 8.14:1 (L1-std gears), 6.75:1 (L2-fast gears), 5.903 (modified L2 16-tooth gear).  
  public static final double kTurningGearRatio = (150.0 / 7.0 / 1.0);     // CALIBRATED = 150.0/7.0.  Mk4i = 150/7 : 1
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4) * 0.9739;     // CALIBRATED ON 1/15/2025. ETU wheels are nominal 4"
  public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
  public static final double kTurningEncoderDegreesPerTick = 360.0 / kEncoderCPR / kTurningGearRatio;
  
  // Robot calibration for feed-forward and max speeds
  public static final double voltageCompSaturation = 12.0;

  // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
  // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
  // and further derate (initial test by 5%) to account for some battery droop under heavy loads.
  // Max speed measured values 3/18/2024:  All 4 motors are 4.17, 4.08, 4.2, 4.09 meters/sec.  So use 4.0 as a conservative value

  public static final double kMaxSpeedMetersPerSecond = 5.1; // CALIBRATED 1/17/2025 5.1 m/s
  public static final double kFullSpeedMetersPerSecond = 0.95 * kMaxSpeedMetersPerSecond;
  public static final double kNominalSpeedMetersPerSecond = 0.5 * kMaxSpeedMetersPerSecond;

  public static final double kFineControlMaxSpeedMetersPerSecond = 1;
  public static final double kFineControlMaxTurningRadiansPerSecond = 1;

  // TODO NOT CALIBRATED
  public static final double kMaxAccelerationMetersPerSecondSquare = 7.5;
  public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
  public static final double kNominalAccelerationMetersPerSecondSquare = 3.5;
  public static final double kMaxTurningRadiansPerSecond = 11.0;
  public static final double kNominalTurningRadiansPerSecond = Math.PI;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;     // Not used in code currently
  public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;

  public static final double kVDriveAvg = 2.350;     // CALIBRATED 1/17/2025 = 2.350.  In voltage per meters per second.
  // TODO NOT CALIBRATED
  private static final double kVmFLrel = 1.0;      // kV modifier for FL drive motor
  private static final double kVmFRrel = 1.0;      // kV modifier for FR drive motor
  private static final double kVmBLrel = 1.0;      // kV modifier for BL drive motor
  private static final double kVmBRrel = 1.0;      // kV modifier for BR drive motor

  // Normalize kVm constants
  private static double kVmAvg = (kVmFLrel + kVmFRrel + kVmBLrel + kVmBRrel) / 4.0;
  public static final double kVmFL = kVmFLrel / kVmAvg;
  public static final double kVmFR = kVmFRrel / kVmAvg;
  public static final double kVmBL = kVmBLrel / kVmAvg;
  public static final double kVmBR = kVmBRrel / kVmAvg;

  public static final double dt = 0.02;       // Timestep for discretizing robot motion, in seconds.  Set this to the scheduler time period = 20ms.
  public static final double kADrive = 0.1203;     // CALIBRATED 1/17/2025 = 0.1203.  In voltage per meters per second squared.
  public static final double kADriveToPose = 0.100;     // In time (seconds) TODO NOT CALIBRATED
  public static final double kSDrive = 0.002;      // CALIBRATED 1/17/2025 = 0.002.  In voltage.
}


  public static final class DriveConstants {
    // The locations of the wheels relative to the physical center of the robot, in meters.
    // X: + = forward.  Y: + = to the left
    // The order in which you pass in the wheel locations is the same order that
    // you will receive the module states when performing inverse kinematics. It is also expected that
    // you pass in the module states in the same order when calling the forward kinematics methods.
    // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
    public static final SwerveDriveKinematics kDriveKinematics =
          new SwerveDriveKinematics(
              new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
              new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
              new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
              new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2));

    // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
    // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
    // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
    // As of Early Jan 2025, make sure to change the networktables JSON file(s) on the roboRIO using FileZilla!
    public static double offsetAngleFrontLeftMotor = 100.18;
    public static double offsetAngleFrontRightMotor = -163.04;
    public static double offsetAngleBackLeftMotor = 19.09;
    public static double offsetAngleBackRightMotor = 2.96;

    // Theta kp value for joystick in rad/sec
    public static final double kPJoystickThetaController = 3;    
  }

  public static class FieldConstants {
    public static final double length = 17.55;      // 57 ft 6 7/8 in = 1755 cm
    public static final double width = 8.05;        // 26 ft 5 in = 805 cm
  }

  // TODO NOT CALIBRATED
  // public static class VisionConstants {
  //   // PhotonVision
  //   public static class PhotonVisionConstants {
  //     public static final int width = 1200;
  //     public static final Transform3d robotToCamFront =
  //             new Transform3d(
  //                 // new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(30.5)),       Changed in B3
  //                 new Translation3d(Units.inchesToMeters(8.9375), Units.inchesToMeters(0), Units.inchesToMeters(25.03125)),
  //                 new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180))); // Cam mounted facing forward in center of robot
  //     public static final String aprilTagCameraName = "AprilTagCamera";
  //     // 1.75 physical center to wheel center
  //     // 16.75 wheel cetner to intake center without bumper
      
  //     public static final Transform3d robotToCamBack =
  //             new Transform3d(
  //                 // new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(30.5)),       Changed in B3
  //                 new Translation3d(Units.inchesToMeters(6.125), Units.inchesToMeters(0), Units.inchesToMeters(25.03125)),
  //                 new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180))); // Cam mounted facing forward in center of robot

  //     public static final String aprilTagCameraBackName = "AprilTagCameraBack";

  //     public static final String noteCameraName = "NoteCamera";
  //     public static final double pitchSetpoint = -18;
  //     public static final double yawSetpoint = 0;
  //   }
  // }

  // TODO NOT CALIBRATED
  public static final class TrajectoryConstants {
    // Max error for robot rotation
    public static final double maxThetaErrorDegrees = 1.0;
    public static final double maxPositionErrorMeters = 0.04; // 1.6 inches

    // Max error for interim positions (not final)
    public static final double interimThetaErrorDegrees = 2.0;        
    public static final double interimPositionErrorMeters = 0.20; // 8 inches

    // TODO Decide on what these values should be Choreo Max error constants
    public static final double xError = 0.25;
    public static final double yError = 0.25;
    public static final double choreoMaxThetaErrorDegrees = 5;
    public static final double endVelocityErrorDegrees = 0.75;

    // Feedback terms for holonomic drive controllers

    // X-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
    public static final double kPXController = 1;

    // Y-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)  
    public static final double kPYController = 1; 

    // Theta-velocity controller:  kp.  Units = (rad/sec of velocity) / (radians of angle error)
    public static final double kPThetaController = 3;

    public static final TrajectoryConfig swerveTrajectoryConfig =
          new TrajectoryConfig(
                  SwerveConstants.kNominalSpeedMetersPerSecond,
                  SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
              .setKinematics(DriveConstants.kDriveKinematics);

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    new TrapezoidProfile.Constraints(
          SwerveConstants.kNominalTurningRadiansPerSecond, SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

    // Constraint for the DriveToPose motion profile for distance being travelled
    public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints =
    new TrapezoidProfileBCR.Constraints(
          SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
  }

  public static final class ElevatorConstants {
    public static final double kEncoderCPR = 2048.0; // Encoder counts per revolution of FalconFX motor pinion gear
    public static final double kElevGearRatio = (9.0 / 1.0); // CALIBRATED. Gear reduction ratio between Kraken and gear driving the elevator
    public static final double kElevPulleyDiameterInches = 1.504; // CALIBRATED. Diameter of the pulley driving the elevator in inches
    public static final double kElevEncoderInchesPerTick = (kElevPulleyDiameterInches * Math.PI) / kEncoderCPR / kElevGearRatio;

    public static final double compensationVoltage = 12.0;

    public static final double maxPercentOutput = 0.1;

    // TODO PARTIALLY CALIBRATED
    public enum ElevatorPosition {
      LOWER_LIMIT(0.0),
      CORAL_HP(0.0),
      CORAL_REEF_L1(14.0),
      CORAL_REEF_L2(19.25),
      CORAL_REEF_L3(35.25), // 35.25 at lab, 36.25 at practice field
      CORAL_REEF_L4(59), // TODO Not calibrated
      UPPER_LIMIT(59.8), 
      SECOND_STAGE(21.0);
  
      public final double value;
  
      ElevatorPosition(double value) {
          this.value = value;
      }
    }
  }

  public static final class EndEffectorConstants {
    public static final double compensationVoltage = 12.0;
    public static final double endEffectorIntakePercent = 0.16;
    public static final double endEffectorOuttakePercent = 0.4;
    public static final double stallThresholdRPM = 100;
    public static final double stallThresholdCurrent = 20;
  }

  public enum BCRColor {
    CANDLE_IDLE(0, 0, 0),             // CANdle
    STICKY_FAULT_PRESENT(255, 0, 0),  // CANdle
    DRIVE_MODE_BRAKE(252, 245, 40),   // CANdle
    DRIVE_MODE_COAST(249, 100, 175);  // CANdle

    public final int r, g, b;
    BCRColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}

public static final class LEDConstants {
      public static final class Patterns {
          // Utilities
          public static final Color[] noPatternStatic = {};
          public static final Color[][] noPatternAnimation = {{}};
      }

      public enum LEDSegmentRange {
          CANdle(0, 8); // Whole CANdle

          public final int index, count;
          LEDSegmentRange(int index, int count) {
              this.index = index;
              this.count = count;
          }
      }
  }
}