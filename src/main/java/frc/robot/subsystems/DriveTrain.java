// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.VisionConstants.PhotonVisionConstants;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.commands.DriveStop;
import frc.robot.utilities.*;

import java.util.ArrayList;
import java.util.List;

// Vision imports
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveTrain extends SubsystemBase implements Loggable {
  private FileLog log;
  private int logRotationKey;
  private boolean fastLogging = false;    // true = enabled to run every cycle, false = follow normal logging cycles
  private final AllianceSelection allianceSelection;

  // Variables for swerve modules
  private final SwerveModule swerveFrontLeft;
  private final SwerveModule swerveFrontRight;
  private final SwerveModule swerveBackLeft;
  private final SwerveModule swerveBackRight;
    
  // Variables for gyro and gyro calibration
  private final Pigeon2 pigeon = new Pigeon2(CANPigeonGyro, Ports.CANDrivetrainBus);
  // private final Pigeon2Configurator pigeonConfigurator = pigeon.getConfigurator();
  // private Pigeon2Configuration pigeonConfig;
  private final StatusSignal<Angle> pigeonYaw = pigeon.getYaw();
  private final StatusSignal<Angle> pigeonPitch = pigeon.getRoll(); // Pigeon is mounted rotated by 90deg, so robot pitch is pigeon roll
  private final StatusSignal<AngularVelocity> pigeonYawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<Boolean> pigeonFault = pigeon.getFault_Hardware();
  private double yawZero = 0.0;
  private double pitchZero = 0.0;
  private final Matrix<N3, N1> closeMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {1,1, Math.PI});
  private final Matrix<N3, N1> farMatrix = new Matrix<>(Nat.N3(), Nat.N1(), new double[] {2,2,2*Math.PI});

  // Variables to help calculate angular velocity for turnGyro
  // private double prevAng;                                            // Last recorded gyro angle
  // private double currAng;                                            // Current recorded gyro angle
  // private double prevTime;                                           // Last time gyro angle was recorded
  // private double currTime;                                           // Current time gyro angle is being recorded
  // private double angularVelocity;                                    // Robot angular velocity in degrees per second
  // private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); // Calculate running average to smooth quantization error in angular velocity calc

  // Variables to store vision cameras
  private PhotonCameraWrapper cameraLeft;
  private PhotonCameraWrapper cameraRight;
  List<PhotonCameraWrapper> cameras = new ArrayList<PhotonCameraWrapper>(2);
  private boolean useVisionForOdometry = false; // false = will not use vision in odometry, true = uses vision for odometry

  private boolean lastCoastReading;

  // Odometry class for tracking robot pose
  private final SwerveDrivePoseEstimator poseEstimator; 
  private final Field2d field = new Field2d();  // Field to display on Shuffleboard

  private double speedAvg;  // Average speed of the robot chassis

  // private boolean elevatorUpPriorIteration = false;  // Slew rate limiter, tracking for elevator position from prior iteration

  // Cache a copy of the DriveStop command for using to cancel the default command (typically DriveWithJoystickAdvanced)
  Command driveStopCommand = new DriveStop(this, log);

  public DriveTrain(AllianceSelection allianceSelection, FileLog log) {
    this.allianceSelection = allianceSelection;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // Create and enable cameras
    cameraLeft = new PhotonCameraWrapper(PhotonVisionConstants.robotToCamLeft, PhotonVisionConstants.leftAprilTagCameraName, log, logRotationKey);
    cameraRight = new PhotonCameraWrapper(PhotonVisionConstants.robotToCamRight, PhotonVisionConstants.rightAprilTagCameraName, log, logRotationKey);
    cameras.add(cameraLeft);
    cameras.add(cameraRight);
    cameraLeft.init();
    cameraRight.init();

    // Create swerve modules
    swerveFrontLeft = new SwerveModule("FL",
      CANDriveFrontLeftMotor, CANDriveTurnFrontLeftMotor, CANTurnEncoderFrontLeft, 
      false, true, false, offsetAngleFrontLeftMotor, SwerveConstants.kVmFL, log);
    swerveFrontRight = new SwerveModule("FR",
      CANDriveFrontRightMotor, CANDriveTurnFrontRightMotor, CANTurnEncoderFrontRight, 
      false, true, false, offsetAngleFrontRightMotor, SwerveConstants.kVmFR, log);
    swerveBackLeft = new SwerveModule("BL",
      CANDriveBackLeftMotor, CANDriveTurnBackLeftMotor, CANTurnEncoderBackLeft, 
      false, true, false, offsetAngleBackLeftMotor, SwerveConstants.kVmBL, log);
    swerveBackRight = new SwerveModule("BR",
      CANDriveBackRightMotor, CANDriveTurnBackRightMotor, CANTurnEncoderBackRight, 
      false, true, false, offsetAngleBackRightMotor, SwerveConstants.kVmBR, log);

    // Sets last coast reading to false (for LED purposes)
    lastCoastReading = false;
    
    // Put drive mode on Shuffleboard
    setDriveModeCoast(false);

    // Configure gyro. This Pigeon is mounted normally, so no need to change orientation
    // pigeonConfig.MountPose.MountPoseYaw = 0;
    // pigeonConfig.MountPose.MountPosePitch = 0;
    // pigeonConfig.MountPose.MountPoseRoll = 0;
    // This Pigeon has no need to trim the gyro
    // pigeonConfig.GyroTrim.GyroScalarX = 0;
    // pigeonConfig.GyroTrim.GyroScalarY = 0;
    // pigeonConfig.GyroTrim.GyroScalarZ = 0;
    // We want the thermal comp and no-motion cal enabled, with the compass disabled for best behavior
    // pigeonConfig.Pigeon2Features.DisableNoMotionCalibration = false;
    // pigeonConfig.Pigeon2Features.DisableTemperatureCompensation = false;
    // pigeonConfig.Pigeon2Features.EnableCompass = false;
    // pigeonConfigurator.apply(pigeonConfig);
    pigeonYawVelocity.setUpdateFrequency(100);

    // Zero gyro
    zeroGyro();

    // Initialize angular velocity variables
    // prevAng = getGyroRaw();
    // currAng = getGyroRaw();
    // prevTime = System.currentTimeMillis();
    // currTime = System.currentTimeMillis();
    // lfRunningAvg.reset();

    // Create and initialize odometery. Set initial location to (0,0) and facing away from driver (regardless of alliance color)
    Pose2d initialPose = new Pose2d(0, 0, allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
    zeroGyroRotation(initialPose.getRotation().getDegrees());
    poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, Rotation2d.fromDegrees(getGyroRotation()), getModulePositions(), initialPose);

    // Register notification for when the alliance changes
    this.allianceSelection.addAllianceChangeNotification(this::resetPoseForAlliance);

    // Create field object on Shuffleboard
    SmartDashboard.putData("Field", field);
  }

  // ************ Gyro methods

  /**
   * Verifies if Gyro is still reading.
   * @return true = gryo is connected to Rio, false = not connected
   */
  public boolean isGyroReading() {
    return !pigeonFault.refresh().getValue();
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * @return raw gyro angle, in degrees (positive = left, negative = right)
   */
  private double getGyroRaw() {
    return pigeonYaw.refresh().getValueAsDouble();
  }

  /**
   * Gets the raw gyro pitch.
	 * @return raw gyro pitch, in degrees, from 180 to -180 (postitive = nose up, negative = nose down)
	 */
	private double getGyroPitchRaw() {
		return pigeonPitch.refresh().getValueAsDouble();
  }

  /**
   * Zeros the gyro pitch in software to the current pitch.
   */
  public void resetGyroPitch() {
    pitchZero = getGyroPitchRaw();
  }

  /**
	 * Zeros the gyro position in software to the current angle.
	 */
	public void zeroGyro() {
    yawZero = getGyroRaw();
    pitchZero = getGyroPitchRaw();
  }
  
  /**
	 * Zeros the gyro position in software against a specified angle.
	 * @param currentHeading current robot angle compared to the zero angle
	 */
	public void zeroGyroRotation(double currentHeading) {
		yawZero = getGyroRaw() - currentHeading;  // Set yawZero to gyro angle, offset to currentHeading
  }

  /**
   * Returns the current calibrated gyro reading. This is based only upon the gyro angle, and 
   * does not take into account the robot rotation on the field as informed by the AprilTags.
   * <p>In general, most code should use driveTrain.getPoseAngle() instead, which uses information
   * from the AprilTags to help correct the robot rotation.
	 * @return gyro angle, in degrees, from 180 to -180 (postitive = left, negative = right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		angle = MathBCR.normalizeAngle(angle);  // Angle is in raw gyro units (-inf,inf), so it need to be converted to (-180, 180]
		return angle;
  }

  /**
   * Gets the gyro pitch.
	 * @return gyro pitch, in degrees, from 180 to -180 (postitive = nose up, negatives = nose down)
	 */
	public double getGyroPitch() {
		return getGyroPitchRaw() - pitchZero;
  }

  /**
   * Gets the angular velocity of the gyro.
   * @return gyro angular velocity, in degrees per second (positive = turning left, negative = turning right)
   */
  public double getAngularVelocity() {
    // TODO check if this is accurate. If so, delete commented out code to calculate angularVelocity
    return pigeonYawVelocity.refresh().getValueAsDouble();
  }

  /**
   * Gets the angular velocity from motor velocity readings, not the gyro.
   * @return angular velocity, in degrees per second (positive = turning left, negative = turning right)
   */
  // public double getAngularVelocityFromWheels() {
  //   // In the 2022 code, this was more accurate than the angular velocity from
  //   // the gyro. This was used in the DriveTurnGyro code. However, angular velocity
  //   // was easy to calculate from a west coast driveTrain. How do we calculate this
  //   // from a swerve drive train? Do we need this method?
  //   return ((getRightEncoderVelocity() - getLeftEncoderVelocity()) / 2) * wheelInchesToGyroDegrees;
  // }

  // ************ Swerve drive methods

  /**
   * Configures the motors and encoders for every swerve module. The swerve modules are 
   * automatically configured in the SwerveModule constructors. So, this method should not need to be called.
   * <p>However, if the robot browns-out or otherwise partially resets, then this can be used to force 
   * the motors and encoders to have the right calibration and settings, especially the calibration angle 
   * for each swerve module.
   * <p><b>NOTE:</b> This procedure includes multiple blocking calls and will delay robot code.
   */
  public void configureSwerveModules() {
    swerveFrontLeft.configSwerveModule();
    swerveFrontRight.configSwerveModule();
    swerveBackLeft.configSwerveModule();
    swerveBackRight.configSwerveModule();
  }

  /**
   * Sets the drive train to coast or brake mode.
   * <p><b>NORE:</b> This procedure includes multiple blocking calls and will delay robot code by ~250ms 
   * if the mode is changed. However, if setCoast is the same as the current setting, then nothing is 
   * sent to the swerve modules and there will not be a delay.
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    swerveFrontLeft.setMotorModeCoast(setCoast);
    swerveFrontRight.setMotorModeCoast(setCoast);
    swerveBackLeft.setMotorModeCoast(setCoast);
    swerveBackRight.setMotorModeCoast(setCoast);

    SmartDashboard.putString("Drive Mode", setCoast ? "Coast" : "Brake");

    if (setCoast && !lastCoastReading) lastCoastReading = true;
    else if (!setCoast && lastCoastReading) lastCoastReading = false;
  }

  /**
   * Gets whether the drive train is in coast mode.
   * @return true = coast mode, false = brake mode
   */
  public boolean isDriveModeCoast() {
    return swerveFrontLeft.isMotorModeCoast() && swerveFrontRight.isMotorModeCoast() && 
      swerveBackLeft.isMotorModeCoast() && swerveBackLeft.isMotorModeCoast();
  }

  /**
   * Sets the drive motors to FOC or trapezoidal commuatation mode.
   * <p><b>NOTE:</b> This takes effect for the <b>next</b> request sent to the motor.
   * @param setFOC true = FOC mode, false = trapezoidal mode
   */
  public void setDriveMotorsFOC(boolean setFOC) {
    swerveFrontLeft.setDriveMotorFOC(setFOC);
    swerveFrontRight.setDriveMotorFOC(setFOC);
    swerveBackLeft.setDriveMotorFOC(setFOC);
    swerveBackRight.setDriveMotorFOC(setFOC);
  }

  /**
   * Sets the percent output of the drive motors.
   * @param percentOutput percent output, -1 to +1
   */
  public void setDriveMotorsOutput(double percentOutput) {
    swerveFrontLeft.setDriveMotorPercentOutput(percentOutput);
    swerveFrontRight.setDriveMotorPercentOutput(percentOutput);
    swerveBackLeft.setDriveMotorPercentOutput(percentOutput);
    swerveBackRight.setDriveMotorPercentOutput(percentOutput);
  }

  /**
   * Sets the percent output of the turning motors.
   * @param percentOutput percent output, -1 to +1
   */
  public void setTurningMotorsOutput(double percentOutput) {
    swerveFrontLeft.setTurnMotorPercentOutput(percentOutput);
    swerveFrontRight.setTurnMotorPercentOutput(percentOutput);
    swerveBackLeft.setTurnMotorPercentOutput(percentOutput);
    swerveBackRight.setTurnMotorPercentOutput(percentOutput);
  }

  /**
   * Sets the facing of the wheels (direction the wheels are pointing).
   * <p><b>NOTE:</b> This sets the absolute facing. It does <b>note</b> turn the wheels to 
   * the "optimized" facing +/-180 degrees if that is closer.
   * @param angle desired wheel facing relative to front of chassis, in degrees, -180 to +180 (positive = left, negative = right, 0 = facing front of robot)
   */
  public void setWheelFacings(double angle) {
    swerveFrontLeft.setWheelFacing(angle);
    swerveFrontRight.setWheelFacing(angle);
    swerveBackLeft.setWheelFacing(angle);
    swerveBackRight.setWheelFacing(angle);
  }

  /**
   * Stops all of the drive and turning motors.
   */
  public void stopMotors() {
    swerveFrontLeft.stopMotors();
    swerveFrontRight.stopMotors();
    swerveBackLeft.stopMotors();
    swerveBackRight.stopMotors();
  }

  /**
   * Sets the states of the swerve modules.
   * @param desiredStates desired swerve module states (0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight)
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    // Convert states to chassisspeeds and slew limit velocities (limit acceleration) to avoid tipping the robot
    ChassisSpeeds chassisSpeeds = kDriveKinematics.toChassisSpeeds(desiredStates);
    double xSlewed, omegaLimited, ySlewed;
    omegaLimited = chassisSpeeds.omegaRadiansPerSecond; // No limiting
    xSlewed = chassisSpeeds.vxMetersPerSecond;          // No limiting
    ySlewed = chassisSpeeds.vyMetersPerSecond;          // No limiting

    // Discretize the movement to avoid unintended robot translation while robot is rotating
    chassisSpeeds = ChassisSpeeds.discretize(xSlewed, ySlewed, omegaLimited, SwerveConstants.dt);

    // Convert back to swerve module states
    desiredStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d());
    
    // Desaturate wheel speeds to a little below max speed. It takes a while to accelerate to
    // max speed, so reducing the max will help movement accuracy.
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    swerveFrontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    swerveFrontRight.setDesiredState(desiredStates[1], isOpenLoop);
    swerveBackLeft.setDesiredState(desiredStates[2], isOpenLoop);
    swerveBackRight.setDesiredState(desiredStates[3], isOpenLoop);
  }

  /**
   * Reads the current swerve module states.
   * @return module states, as measured by the encoders (0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight)
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      swerveFrontLeft.getState(), swerveFrontRight.getState(),
      swerveBackLeft.getState(), swerveBackRight.getState()
    };
  }

  /**
   * Returns the speed of the chassis in X, Y, and theta <b>in the robot frame of reference</b>.
   * <p>Speed of the robot in the x direction, in meters per second (positive = forward)
   * <p>Speed of the robot in the y direction, in meters per second (positive = move to the left)
   * <p>Angular rate of the robot, in radians per second (positive = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Reads the current swerve module positions.
   * @return module positions, as measured by the encoders (0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight)
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      swerveFrontLeft.getPosition(), swerveFrontRight.getPosition(),
      swerveBackLeft.getPosition(), swerveBackRight.getPosition()
    };
  }
  
  /**
   * Drives the robot using desired robot velocity and orientation, such as from joystick info.
   * @param xSpeed speed of the robot in the x direction, in meters/second (positive = move towards red driver station)
   * @param ySpeed speed of the robot in the y direction, in meters/second (positive = move left in reference to the blue driver station)
   * @param rot angular rate of the robot, in radians/second (positive = turn to the left)
   * @param fieldRelative true = the provided x and y speeds are relative to the field, false = the provided x and y speeds are relative to the current facing of the robot
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
    drive(xSpeed, ySpeed, rot, new Translation2d(), fieldRelative, isOpenLoop);
  }

  /**
   * Drives the robot using desired robot velocity and orientation, such as from joystick info.
   * @param xSpeed speed of the robot in the x direction, in meters/second (positive = move towards red driver station)
   * @param ySpeed speed of the robot in the y direction, in meters/second (positive = move left in reference to the blue driver station)
   * @param rot angular rate of the robot, in radians/second (positive = turn to the left)
   * @param centerOfRotationMeters center of rotation
   *  <p> For example, if you set the center of rotation at one corner of the robot and XSpeed, YSpeed = 0, then the robot will 
   *  rotate around that corner. This feature may not work if fieldRelative = true (need to test)
   * @param fieldRelative true = the provided x and y speeds are relative to the field, false = the provided x and y speeds are relative to the current facing of the robot
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
   public void drive(double xSpeed, double ySpeed, double rot, Translation2d centerOfRotationMeters, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds chassisSpeed;
    chassisSpeed = fieldRelative 
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroRotation()))
      : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeed, centerOfRotationMeters);
    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  // ************ Odometry methods

  /**
   * Gets the currently-estimated position of the robot on the field.
   * @return robot pose on the field. Pose components include:
   *  <p> Robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   *  <p> Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   *  <p> Robot angle on the field (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the currently-estimated angle of the robot on the field.
   * @return robot angle on the field, in degrees (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   */
  public double getPoseAngle() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Resets the pose estimator to the specified pose (defines the robot's position and orientation on the field).
   * This method also resets the gyro, which is required for the pose to properly reset.
   * @param pose pose to set the pose estimator. Pose components include:
   *  <p> Robot X location in the field, in meters (0 = field edge in front of the Blue driver station, positive = away from the Blue drivestation)
   *  <p> Robot Y location in the field, in meters (0 = right edge of field when standing in the Blue driver station, positive = left when looking from the Blue drivestation)
   *  <p> Robot angle on the field (0 = facing away from the Blue drivestation, positive = to the left, negative = to the right)
   */
  public void resetPose(Pose2d pose) {
    zeroGyroRotation(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions(), pose);
  }

  /**
   * Resetter used in alliance selection to properly reset the angle when the alliance of the robot changes.
   * @param alliance new alliance of the robot
   */
  public void resetPoseForAlliance(Alliance alliance) {
    Pose2d pose = getPose();

    // TODO check if this works correctly! It should work if the DriverStation is set to Red and Shuffleboard = Auto Alliance when the robot boots
    // And it should still work if the alliance changes on Shuffleboard.

    // When the alliance changes (Blue -> Red or Red -> Blue), rotate the robot orientation by 180 degrees.
    // However, only reset pose if robot is not in the valid field area (such as when robot code boots).
    // Otherwise, the robot is on the field, so don't change anything regardless of the alliance change.
    if (pose.getY() < 0.2 && pose.getX() < 0.2) {
      double rotation = MathBCR.normalizeAngle(pose.getRotation().getDegrees() - 180);
      resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(rotation)));

      // DriveWithJoystickAdvanced will try to turn the robot 180 degrees to maintain the robot heading.
      // Run the DriveStop command to interrupt DriveWithJoystickAdvanced.
      driveStopCommand.schedule();
    }
  }
  
  /**
   * Returns the speed of the robot in X, Y, and theta <b>in the field frame of reference</b>.
   * <p> Speed of the robot in the x direction, in meters per second (+ = away from the Blue drivestation)
   * <p> Speed of the robot in the y direction, in meters per second (+ = left when looking from the Blue drivestation)
   * <p> Angular rate of the robot, in radians per second (+ = turn to the left)
   * @return ChassisSpeeds object representing the chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), Rotation2d.fromDegrees(getGyroRotation()));
  }

  // ************ Information methods

  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Checks if the CANbus and gyro are working. Sometimes, when the robot boots up, either the CANbus or
   * the gyro don't initialize properly. ROBOT CODE WILL NOT BE ABLE TO CONTROL MOTORS when this happens, so
   * always check this before starting a match!
   * @return true = something is not working, false = CANbus and gyro are both working
   */
  public boolean canBusError() {
    return ((swerveFrontLeft.getDriveBusVoltage() < 7.0) || (swerveFrontLeft.getDriveTemp() < 5.0) || !isGyroReading());
  }


  @Override
  public void periodic() {    
    // Save current angle and time for calculating angVel
    // currAng = getGyroRaw();
    // currTime = System.currentTimeMillis();
 
    // Calculate angVel in degrees per second
    // angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );

    updateOdometry();
    
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      // updateDriveLog(false);

      if(!isGyroReading()) RobotPreferences.recordStickyFaults("Gyro", log);

      ChassisSpeeds robotSpeeds = getRobotSpeeds();

      // SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      SmartDashboard.putNumber("Drive Speed", speedAvg);
      SmartDashboard.putNumber("Drive X Velocity", robotSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Drive Y Velocity", robotSpeeds.vyMetersPerSecond);
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putBoolean("Drive isBrakeMode", !isDriveModeCoast());
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putNumber("Drive Gyro Rotation", getGyroRotation());
      SmartDashboard.putNumber("Drive AngVel", getAngularVelocity());
      SmartDashboard.putNumber("Drive Pitch", getGyroPitch());
      
      // Position from poseEstimator (helpful for auto routines)
      Pose2d pose = poseEstimator.getEstimatedPosition();
      SmartDashboard.putNumber("Drive Odometry X", pose.getTranslation().getX());
      SmartDashboard.putNumber("Drive Odometry Y", pose.getTranslation().getY());
      SmartDashboard.putNumber("Drive Odometry Theta", pose.getRotation().getDegrees());

      // TODO This code creates errors after running for a few minutes. Need to debug.
      // SmartDashboard.putData("Swerve Drive", new Sendable() {
      //   @Override
      //   public void initSendable(SendableBuilder builder) {
      //     builder.setSmartDashboardType("SwerveDrive");

      //     builder.addDoubleProperty("Front Left Angle", () -> swerveFrontLeft.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Front Left Velocity", () -> swerveFrontLeft.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Front Right Angle", () -> swerveFrontRight.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Front Right Velocity", () -> swerveFrontRight.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Back Left Angle", () -> swerveBackLeft.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Back Left Velocity", () -> swerveBackLeft.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Back Right Angle", () -> swerveBackRight.getTurningEncoderDegrees()*(Math.PI/180), null);
      //     builder.addDoubleProperty("Back Right Velocity", () -> swerveBackRight.getDriveEncoderVelocity(), null);

      //     builder.addDoubleProperty("Robot Angle", () -> getGyroRotation()*(Math.PI/180), null);
      //   }
      // });

      // Using vision to update odometry
      SmartDashboard.putBoolean("Vision Updating Odometry", useVisionForOdometry);

      // Values from each swerve module
      swerveFrontLeft.updateShuffleboard();
      swerveFrontRight.updateShuffleboard();
      swerveBackLeft.updateShuffleboard();
      swerveBackRight.updateShuffleboard();

      // Values for bug fixing
      SmartDashboard.putNumber("Drive Bus Volt", swerveFrontLeft.getDriveBusVoltage());
    }

    // Save current angVel values as previous values for next calculation
    // prevAng = currAng;
    // prevTime = currTime; 
  }

  /**
   * Writes information about the driveTrain to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    Pose2d pose = poseEstimator.getEstimatedPosition();
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    log.writeLog(logWhenDisabled, "Drive", "Update Variables", 
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), 
      "Gyro Velocity", getAngularVelocity(), "Pitch", getGyroPitch(), 
      "Odometry X", pose.getTranslation().getX(), "Odometry Y", pose.getTranslation().getY(), 
      "Odometry Theta", pose.getRotation().getDegrees(),
      "Drive Speed", speedAvg,
      "Drive X Velocity", robotSpeeds.vxMetersPerSecond, 
      "Drive Y Velocity", robotSpeeds.vyMetersPerSecond,
      "Bus voltage", swerveFrontLeft.getDriveBusVoltage(),
      swerveFrontLeft.getLogString(),
      swerveFrontRight.getLogString(),
      swerveBackLeft.getLogString(),
      swerveBackRight.getLogString()
    );
  }

  /**
   * Gets a string with the absolute value of the output percentages of all 4 swerve drive motors, formatted for logging.
   * @return string for logging
   */
  public String getDriveModuleOutputs() {
    return StringUtil.buildStringWithCommas(
      "FL drive output", Math.abs(swerveFrontLeft.getDriveOutputPercent()),
      "FR drive output", Math.abs(swerveFrontRight.getDriveOutputPercent()),
      "BL drive output", Math.abs(swerveBackLeft.getDriveOutputPercent()),
      "BR drive output", Math.abs(swerveBackRight.getDriveOutputPercent()),
      "Bus voltage", swerveFrontLeft.getDriveBusVoltage()
    );
  }

  // ************ Camera methods

  /**
   * Sets whether vision is used in odometry.
   * @param enabled true = uses vision for odometry, false = does not use vision for odometry
   */
  public void setVisionForOdometryState(boolean enabled) {
    useVisionForOdometry = enabled;
  }

  /**
   * Calculates a pose estimator standard deviation matrix based the distance to the AprilTag.
   * Increases the standard deviation when the robot is farther from the AprilTag (and hence the camera is less accurate).
   * @param xDist X distance to AprilTag
   * @param yDist Y distance to AprilTag
   * @return pose estimator standard deviation matrix
   */
  public Matrix<N3, N1> getVisionMatrix(double xDist, double yDist){
    double distance = Math.hypot(xDist, yDist);
    double mulFactor = Math.max(1, distance/3.0);
    return closeMatrix.times(mulFactor);
  }

  /**
   * Updates odometry based on wheel movement, gyro, and camera/AprilTags.
   */
  public void updateOdometry() {
    // Update odometry based on wheel movement and gyro
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()), getModulePositions());

    // Update odometry based on cameras
    for (PhotonCameraWrapper camera : cameras) {
      if (camera.hasInit()) {
        PhotonPipelineResult camResult = camera.getLatestResult();
        if (camResult != null) {
          Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition(), camResult);
          if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            SmartDashboard.putNumber("Vision " + camera.getCameraName() + " X", camPose.estimatedPose.toPose2d().getX());
            SmartDashboard.putNumber("Vision " + camera.getCameraName() + " Y", camPose.estimatedPose.toPose2d().getY());
            SmartDashboard.putNumber("Vision " + camera.getCameraName() + " rot", camPose.estimatedPose.toPose2d().getRotation().getDegrees());
  
            // Only run camera updates for pose estimator if useVisionForOdometry is true
            if (camResult.hasTargets() && useVisionForOdometry) {
              PhotonTrackedTarget bestTarget = camResult.getBestTarget();
              if (bestTarget.getBestCameraToTarget().getX() < 1.5) {
                poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, closeMatrix);
              } else if (bestTarget.getBestCameraToTarget().getX() < 3) {
                poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, farMatrix);
              }
            }
          }
        }
      }
    }
    
    // Update robot average speed
    ChassisSpeeds robotSpeeds = getRobotSpeeds();
    speedAvg = Math.hypot(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond);

    // Place robot on field object
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }  
}
