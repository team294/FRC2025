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
import edu.wpi.first.wpilibj.util.Color;
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
  public static final String bcrRobotCodeVersion = "B1";

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
  
  public static final class RobotDimensions {
    // Drivebase adjustment for path-of-wheel diameter when turning in place
    private static final double DrivetrainAdjustmentFactor = 1.0071;      // CALIBRATED

    // Left-right distance between the drivetrain wheels, measured from center to center, in meters
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.75) * DrivetrainAdjustmentFactor;

    // Front-back distance between the drivetrain wheels, measured from center to center, in meters
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.75) * DrivetrainAdjustmentFactor;
    
    // Width of robot in meters plus bumpers, in meters
    public static final double robotWidth = Units.inchesToMeters(35.5);   // CALIBRATED (for DriveToReef)

    // Length of the robot in meters plus bumpers, in meters
    public static final double robotLength = Units.inchesToMeters(36.0);

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
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4) * 0.9554;        // CALIBRATED Wheels are nominal 4".  LAR was 0.9683, AVR is 0.9554
    public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
    public static final double kTurningEncoderDegreesPerTick = 360.0 / kEncoderCPR / kTurningGearRatio;
  
    // Robot calibration for feed-forward and max speeds
    public static final double voltageCompSaturation = 12.0;

    // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
    // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
    // and further derate (initial test by 5%) to account for some battery droop under heavy loads.

    public static final double kMaxSpeedMetersPerSecond = 4.8;  // CALIBRATED
    public static final double kFullSpeedMetersPerSecond = 0.95 * kMaxSpeedMetersPerSecond;
    public static final double kNominalSpeedMetersPerSecond = 0.5 * kMaxSpeedMetersPerSecond;

    public static final double kFineControlMaxSpeedMetersPerSecond = 1;
    public static final double kFineControlMaxTurningRadiansPerSecond = 1;

    // CALIBRATED
    public static final double kMaxAccelerationMetersPerSecondSquare = 6.0;
    public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
    public static final double kNominalAccelerationMetersPerSecondSquare = 3.5;
    public static final double kMaxTurningRadiansPerSecond = 11.0;
    public static final double kNominalTurningRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;     // Not used in code currently
    public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;

    // CALIBRATED
    public static final double kVDriveAvg = 2.344;  // In voltage per meters/second    LAR was 2.251 (FOC off), FOC cal is 2.392 (FOC on), turn for AVR is 2.344 (FOC on)
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
    public static final double kADrive = 0.2383;      // CALIBRATED In voltage per meters per second^2   LAR was 0.2692 (FOC off), AVR is 0.2383 (FOC on)
    public static final double kADriveToPose = 0.050; // CALIBRATED on ETU in 3/2/2025 In seconds (On the ETU, DriveToPose behaves better with a small value for kADriveToPose)
    public static final double kSDrive = 0.141;       // CALIBRATED In voltage      LAR was 0.030 (FOC off), AVR is 0.141 (FOC on)

    // Minimum abs delta (in m/sec) between actual wheel velocity and desired wheel velocity for kADrive to be applied.
    // If the delta is less than this, then don't use kADrive. This prevents the drive motors from jittering.
    // Note that the swerve module kP in the velocity controller will still work to maintain the proper wheel speed.
    public static final double velMinDeltaUsingkA = 0.1;        // For LAR was 0.3, for AVR is 0.1
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

    // Distance bumpers should be away from the reef
    // This distance is the where the scoring prep sequence will take place for L1-L3,
    // along with is how far the robot should be located to score on L4.
    public static final double distanceFromReefToScore = Units.inchesToMeters(6.25);  // was 0.25 meters, changed to be scoring location for L4

    // Distance bumpers should be away from the reef
    // This distance is where the algae intake prep will take place for both levels
    public static final double distanceFromReefToPickupAlgae = Units.inchesToMeters(6.25);  // TODO: Calibrate

    // Back offset for robot to pick up algae, in meters
    public static final double ReefAlgaePickupPositionOffset = 0.2;
  }

  public static class FieldConstants {
    public static final double length = 17.548;  // CALIBRATED 57 ft 6 7/8 in = 1754.8 cm
    public static final double width = 8.052;    // CALIBRATED 26 ft 5 in = 805.2 cm

    public static enum ReefLevel {
      L1, L2, L3, L4
    }

    public static enum ReefLocation {
      A(false, false), 
      B(true, false), 
      C(false, true), 
      D(true, true), 
      E(false, false), 
      F(true, false), 
      G(false, true), 
      H(true, true), 
      I(false, false), 
      J(true, false), 
      K(false, true), 
      L(true, true);

      public final boolean onRightSide;
      public final boolean isAlgaeLower;

      ReefLocation(boolean onRightSide, boolean isAlgaeLower) {
        this.onRightSide = onRightSide;
        this.isAlgaeLower = isAlgaeLower;
      }
    }

    public static enum AlgaeLocation {
      AB(false), 
      CD(true), 
      EF(false), 
      GH(true), 
      IJ(false), 
      KL(true);

      public final boolean isLower;

      AlgaeLocation(boolean isLower) {
        this.isLower = isLower;
      }
    }

    // Calculated by measuring y distance between center of reef wall and reef pole (6.469731 in), converted to meters
    public static final double ReefScoringPositionAprilTagOffset = 0.164331496063;

    public static final double bargeScorableWidth = 3.5;  // Width of alliance barge structure (not net) is 3.72 meters, reduced for consistency
    public static final double bargeScoringOffset = 1.32; // CALIBRATED 3/29/2025, was 1.07 in lab, changed at AVR
  }

  public static class VisionConstants {
    public static class PhotonVisionConstants {        
      public static final Transform3d robotToCamLeft = new Transform3d(
        new Translation3d(Units.inchesToMeters(9.72 - 0.8 + 0.6), Units.inchesToMeters(12.02 + 0.4 - 0.5), Units.inchesToMeters(12.7204)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(-34.5))); // Cam mounted facing forward-right on the left side of the robot
      public static final Transform3d robotToCamRight = new Transform3d(
        new Translation3d(Units.inchesToMeters(9.72 + 0.8 - 0.6), Units.inchesToMeters(-12.02 - 0.4 - 0.5), Units.inchesToMeters(12.7204)),
        new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(30.5 + 1.0))); // Cam mounted facing forward-left on the right side of the robot

      public static final String leftAprilTagCameraName = "LeftCamera";
      public static final String rightAprilTagCameraName = "RightCamera";
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
    public static final double kPXController = 10;     // was 4

    // Y-velocity controller kP. Units = (meters/sec of velocity) / (meters of position error)  
    public static final double kPYController = 10;     // was 4

    // Theta-velocity controller kP. Units = (rad/sec of velocity) / (radians of angle error)
    public static final double kPThetaController = 10;   // was 3

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
    public static final double intakePercent = 0.45;
    public static final double reverseIntakePercent = -0.2;
  }

  public static final class CoralEffectorConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.1;       // CALIBRATED
    public static final double fastIntakePercent = 0.25;  // CALIBRATED for LAR
    public static final double outtakePercent = 0.6;      // CALIBRATED

    public static final double centerRotationsUndershoot = 1.625;  // CALIBRATED #3  Measure the typical undershoot with kP.  This value (in motor rotataions) will be added to the encoder reading when the coral sensor is triggered.
    public static final double centeringTolerance = 0.07;  // CALIBRATED #1  Position tolerance (in rotations) for holding coral [smaller than 1/2 of the position window where both sensors see the coral]
    public static final double centeringStepSize = centeringTolerance * 0.5; // #4 If the autoHold oscillates (not setPosition oscillating), then reduce this step size.

    public static final double kP = 5.0;    // CALIBRATE #2 (in Phoenix tuner)   kP = (desired-output-volts) / (error-in-encoder-rotations)
    public static final double kI = 0.0;    // CALIBRATED
    public static final double kD = 0.0;    // CALIBRATED
    public static final double kG = 0.0;    // CALIBRATED #5 if needed.  (start with +0.009 * 12 = +0.108 ?)  Or measure holding voltage to keep coral when vertical (and kP turned off)
          //  #6 For kG, add an offset angle to true vertical coral, if needed.  (in coral.setposition inside the cosine function)
  }

  public static final class AlgaeGrabberConstants {
    public static final double compensationVoltage = 12.0;
    public static final double intakePercent = 0.4;             // CALIBRATED 3/29
    public static final double netOuttakePercent = -0.50;       // CALIBRATED 3/29
    public static final double processorOuttakePercent = -0.15; // CALIBRATED 3/29
  }

  public static final class ElevatorConstants {
    public static final double kEncoderCPR = 1.0;                 // Encoder counts per revolution of the motor pinion gear
    public static final double kElevGearRatio = (5.0 / 1.0);      // Gear reduction ratio between Kraken and gear driving the elevator CALIBRATED FOR 2025 (5:1)
    public static final double kElevPulleyDiameterInches = 1.504; // Diameter of the pulley driving the elevator in inches (1.504" nominal)  GOAL HIEGHTS CALIBRATED USING THIS VALUE
    public static final double kElevEncoderInchesPerTick = (kElevPulleyDiameterInches * Math.PI) / kEncoderCPR / kElevGearRatio;
    
    public static final double compensationVoltage = 12.0;

    public static final double maxUncalibratedPercentOutput = 0.05;
    public static final double maxManualPercentOutput = 0.1;  // Max elevator speed when driven using Xbox controller
    public static final double maxPercentOutput = 1.0;        // Absolute max output to elevator motors

    // CALIBRATED
    public enum ElevatorPosition {
      LOWER_LIMIT(0.0),
      UPPER_LIMIT(75.0);
 
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
    public static final double maxUncalibratedPercentOutput = 0.1;  // CALIBRATED
    public static final double maxManualPercentOutput = 0.05;       // CALIBRATED
    public static final double maxPercentOutput = 0.2;              // TODO CALIBRATE FOR 2025

    // Should be updated in RobotPreferences, so it cannot be final
    // When wrist CG (with coral) is vertical, wrist angle should read 90.0 degrees.
    // When wrist coral top metal plate is horizontal (bubble level), wrist angle should read 100.6 degrees
    public static double offsetAngleCANcoder = -31.5;                 // CALIBRATED.   CANCoder raw angle (in degrees) when arm is at 0 degrees.

    // 1 makes absolute position unsigned [0, 1); 0.5 makes it signed [-0.5, 0.5), 0 makes it always negative
    public static double cancoderDiscontinuityPoint = 0.50;          // CALIBRATED - should be the center of the region of unallowed motion

    public static final double kP = 40.0;    // CALIBRATED    kP = (desired-output-volts) / (error-in-wrist-rotations)
    public static final double kI = 0.0;    // CALIBRATED
    public static final double kD = 0.0;    // CALIBRATED
    public static final double kG = 0.522;   // CALIBRATED    kG = Feed foward voltage to add to hold wrist horizontal (0 deg)
    public static final double kS = 0.17;   // CALIBRATED    kS = (volts)
    public static final double kV = 2.66;   // CALIBRATED    kV = (volts)/(wrist-rotations/sec)

    public static final double MMCruiseVelocity = 0.80;                   // Max velocity in wrist rotations / second CALIBRATED  TODO opportunity to make this faster
    public static final double MMAcceleration = MMCruiseVelocity / 0.35;  // Max acceleration in wrist rotations / second^2. MMVel / MMAccel = seconds to full velocity. CALIBRATED
    public static final double MMJerk = MMAcceleration / 0.05;            // Max jerk in wrist rotations / second^3. MMAccel / MMJerk = seconds to full acceleration. CALIBRATED

    // CALIBRATED
    public enum WristAngle {
      LOWER_LIMIT(-13.0),  // Wrist chin strap limit (without coral = -26.5 deg).  Elevator needs to be above 5.4" to get to this angle.
      UPPER_LIMIT(100.5);

      @SuppressWarnings({"MemberName", "PMD.SingularField"})
      public final double value;
      WristAngle(double value) { this.value = value; }
    }
  }

  public static final class ElevatorWristConstants {
    public enum ElevatorWristPosition {
      START_CONFIG(0.0, 100.0),

      CORAL_HP(0.0, 82.0),

      CORAL_L1(13.0, 95.0),
      CORAL_L2(25.56, 65.0),
      CORAL_L3(40.28, 65.0),
      CORAL_L4(71.0, 57.0),
      CORAL_L4_COPANEL(71.0, 28.0),  //stop  meas = 71 28   CAD = 70.7, 30 TODO change angle to ~57, do not be fully up against reef when scoring (2 inches off), and recheck regions

      ALGAE_GROUND(5.8, -6.5),
      ALGAE_LOWER(22.3, 24.0),
      ALGAE_UPPER(38.0, 24.0),
      ALGAE_LOLLIPOP(12.0, 10.0),

      ALGAE_PROCESSOR(9.84, 10.0),
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
    public static final double kClimberGearRatio = (263.0/1.0);          // CALIBRATED FOR 2025  A4:  Was 135:1, now 263:1
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
    public static final double kV = 15.2 * 263.0/135.0;    // CALIBRATED   kV = (volts)/(wrist-rotations/sec)

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

  // Colors for the LEDs based on different robot states (see BCRRobotState)
  public enum BCRColor {
    CANDLE_IDLE(0, 0, 0),              // CANdle Black (off)
    CANDLE_STICKY_FAULT(255, 0, 0),    // CANdle Red
    NEUTRAL(0, 0, 0),                  // Black (off)
    ALGAE_MODE(0, 200, 255),           // Turquoise
    CORAL_MODE(255, 0, 255),           // Purple
    AUTO_DRIVE_COMPLETE(0, 255, 0),    // Green
    SUBSYSTEM_UNCALIBRATED(255, 0, 0), // Red
    SCORING_COMPLETE(0, 255, 0),       // Green
    BLUE(19, 82, 188),                 // Blue
    ORANGE(240, 107, 14),              // Orange
    WHITE(255, 255, 255);              // White

    public final int r, g, b;
    BCRColor(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public static final class LEDConstants {
    public static int ledPerGap = 2;

    public static final class EmptyPatterns {
      public static final Color[] noPatternStatic = {};
      public static final Color[][] noPatternAnimation = {{}};
    }

    public enum LEDSegmentRange {
      CANdle(0, 8),
      StripRight(CANdle.count, 40),
      StripHorizontal(StripRight.index + StripRight.count, 40),
      StripLeft(StripHorizontal.index + StripHorizontal.count, 40),
      StripAll(StripRight.index, StripRight.count + StripHorizontal.count + StripLeft.count),
      Full(0, CANdle.count + StripRight.count + StripHorizontal.count + StripLeft.count);


      public final int index, count;
      LEDSegmentRange(int index, int count) {
        this.index = index;
        this.count = count;
      }
    }
  }
}