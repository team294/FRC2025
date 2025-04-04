// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;
import frc.robot.Constants.*;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.LED.StripEvents;

public class Wrist extends SubsystemBase implements Loggable {
  
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboard
  private LED led;

  // Create wrist motor
  private final TalonFX wristMotor = new TalonFX(Ports.CANWrist);
  private final TalonFXConfigurator wristMotorConfigurator = wristMotor.getConfigurator();

  // We create two configs for the wrist motor -- one with RemoteCANcoder and one with RotorEncoder.
  // The default config is RemoteCANcoder. If the CANcoder fails, the wrist will the
  // use the RotorEncoder config as a fallback.
  private boolean usingCANcoder;                              // true = using CANcoder configuration, false = using RotorEncoder configuration
  private TalonFXConfiguration wristMotor_RotorEncoderConfig;  // Configuration reading the rotor encoder
  private TalonFXConfiguration wristMotor_CANcoderConfig;      // Configuration reading the CANcoder

  private VoltageOut wristVoltageControl = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage wristPositionControl = new PositionVoltage(0.0).withEnableFOC(true);
  private MotionMagicVoltage wristMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  // Create variables for the wrist Kraken motor
  private final StatusSignal<Voltage> wristSupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> wristTemp;       // Motor temp, in degrees Celsius
  private final StatusSignal<Double> wristDutyCycle;       // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> wristStatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> wristEncoderPosition;  // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> wristEncoderVelocity;
  // private final StatusSignal<Voltage> wristVoltage;
  private final StatusSignal<ControlModeValue> wristControlMode;
  // private final StatusSignal<AngularAcceleration> wristEncoderAcceleration;

  // Create CANcoder
  private final CANcoder canCoder = new CANcoder(Ports.CANWristEncoder);
  private final CANcoderConfigurator canCoderConfigurator = canCoder.getConfigurator();
  private CANcoderConfiguration canCoderConfig;

  // CANcoder signals and sensors
  private final StatusSignal<MagnetHealthValue> canCoderMagnetHealth;
  private final StatusSignal<Angle> canCoderPosition;            // CANcoder position, in CANcoder rotations

  // Create Data Log Entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogWristTemp = new DoubleLogEntry(log, "/Wrist/Motor Temp");
  private final DoubleLogEntry dLogWristPctOut = new DoubleLogEntry(log, "/Wrist/Pct Out");
  private final DoubleLogEntry dLogWristCurrent = new DoubleLogEntry(log, "/Wrist/Current (amps)");
  // private final DoubleLogEntry dLogWristVoltage = new DoubleLogEntry(log, "/Wrist/Wrist Voltage");
  private final DoubleLogEntry dLogBatteryVoltage = new DoubleLogEntry(log, "/Wrist/Battery Voltage");
  private final DoubleLogEntry dLogCANcoderRotRaw = new DoubleLogEntry(log, "/Wrist/CANcoder Rot Raw");
  // private final DoubleLogEntry dLogWristEncoderRotRaw = new DoubleLogEntry(log, "/Wrist/Encoder Rot Raw");
  private final DoubleLogEntry dLogWristCalZero = new DoubleLogEntry(log, "/Wrist/WristCalZero");
  private final DoubleLogEntry dLogWristTarget = new DoubleLogEntry(log, "/Wrist/Target");
  private final DoubleLogEntry dLogWristEncAngle = new DoubleLogEntry(log, "/Wrist/Encoder Angle (deg)");
  private final DoubleLogEntry dLogWristAngle = new DoubleLogEntry(log, "/Wrist/Wrist Angle (Deg)");
  private final DoubleLogEntry dLogWristVelDPS = new DoubleLogEntry(log, "/Wrist/Wrist Velocity dps");
  // private final DoubleLogEntry dLogWristAccelDPS2 = new DoubleLogEntry(log, "/Wrist/Wrist Acceleration dps");
  private final BooleanLogEntry dLogWristCalibrated = new BooleanLogEntry(log, "/Wrist/Wrist Calibrated?");
  private final BooleanLogEntry dLogCANcoderConnected = new BooleanLogEntry(log, "/Wrist/CANcoder Connected?");

  private double encoderZero = 0;   // Reference raw encoder angle for wrist encoder (may be motor built-in encoder or remote CANcoder) at 0 degrees.

  private boolean wristCalibrated = false;                // Default to wrist being uncalibrated. Calibrate from robot preferences or "Calibrate Wrist Zero" button on dashboard.
  private boolean lastCalibrationState = wristCalibrated; // Store last calibration state for LEDs

  private boolean sent = false; // true = uncal. state sent to LEDs

  private double safeAngle; // Current wrist target on position control on the motor (if in position mode)

  public Wrist(String subsystemName, LED led) {
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.subsystemName = subsystemName;
    this.led = led;

    // Get signal and sensor objects for motor
    wristSupplyVoltage = wristMotor.getSupplyVoltage();
    wristTemp = wristMotor.getDeviceTemp();
    wristDutyCycle = wristMotor.getDutyCycle();
    wristStatorCurrent = wristMotor.getStatorCurrent();
    wristEncoderPosition = wristMotor.getPosition();
    wristEncoderVelocity = wristMotor.getVelocity();
    // wristVoltage = wristMotor.getMotorVoltage();
    wristControlMode = wristMotor.getControlMode();
    // wristEncoderAcceleration = wristMotor.getAcceleration();

    // Create the motor configuration for using the rotor encoder
    wristMotor_RotorEncoderConfig = new TalonFXConfiguration();
    wristMotor_RotorEncoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;         // CALIBRATED
    wristMotor_RotorEncoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotor_RotorEncoderConfig.Voltage.PeakForwardVoltage = WristConstants.compensationVoltage;
    wristMotor_RotorEncoderConfig.Voltage.PeakReverseVoltage = -WristConstants.compensationVoltage;
    wristMotor_RotorEncoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    wristMotor_RotorEncoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Turn off soft limits until encoder is calibrated
    wristMotor_RotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    wristMotor_RotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    wristMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    wristMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    wristMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    wristMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure PID for MotionMagicVoltage control
    wristMotor_RotorEncoderConfig.Slot0.kP = WristConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
    wristMotor_RotorEncoderConfig.Slot0.kI = 0.0;
    wristMotor_RotorEncoderConfig.Slot0.kD = 0.0;
    wristMotor_RotorEncoderConfig.Slot0.kS = WristConstants.kS;
    wristMotor_RotorEncoderConfig.Slot0.kV = WristConstants.kV;
    wristMotor_RotorEncoderConfig.Slot0.kA = 0.0;

    // Configure Magic Motion settings
    wristMotor_RotorEncoderConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMCruiseVelocity;
    wristMotor_RotorEncoderConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MMAcceleration;
    wristMotor_RotorEncoderConfig.MotionMagic.MotionMagicJerk = WristConstants.MMJerk;

    // Configure encoder to user for feedback
    wristMotor_RotorEncoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    wristMotor_RotorEncoderConfig.Feedback.RotorToSensorRatio = 1.0;
    wristMotor_RotorEncoderConfig.Feedback.SensorToMechanismRatio = WristConstants.kWristGearRatio;
    wristMotor_RotorEncoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    wristPositionControl.Slot = 0;
    wristPositionControl.OverrideBrakeDurNeutral = true;
    wristMMVoltageControl.Slot = 0;
    wristMMVoltageControl.OverrideBrakeDurNeutral = true;

    // Create the motor configuration for using the CANcoder
    wristMotor_CANcoderConfig = new TalonFXConfiguration();
    wristMotor_CANcoderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;         // CALIBRATED
    wristMotor_CANcoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotor_CANcoderConfig.Voltage.PeakForwardVoltage = WristConstants.compensationVoltage;
    wristMotor_CANcoderConfig.Voltage.PeakReverseVoltage = -WristConstants.compensationVoltage;
    wristMotor_CANcoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    wristMotor_CANcoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Turn off soft limits until CANcoder is calibrated
    wristMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    wristMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    wristMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    wristMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    wristMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    wristMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristMotor_CANcoderConfig.Slot0.kP = WristConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
    wristMotor_CANcoderConfig.Slot0.kI = 0.0;
    wristMotor_CANcoderConfig.Slot0.kD = 0.0;
    wristMotor_CANcoderConfig.Slot0.kS = WristConstants.kS;
    wristMotor_CANcoderConfig.Slot0.kV = WristConstants.kV;
    wristMotor_CANcoderConfig.Slot0.kA = 0.0;

    // Configure Magic Motion settings
    wristMotor_CANcoderConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMCruiseVelocity;
    wristMotor_CANcoderConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MMAcceleration;
    wristMotor_CANcoderConfig.MotionMagic.MotionMagicJerk = WristConstants.MMJerk;

    // Configure encoder to user for feedback
    wristMotor_CANcoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristMotor_CANcoderConfig.Feedback.FeedbackRemoteSensorID = Ports.CANWristEncoder;
    wristMotor_CANcoderConfig.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;
    wristMotor_CANcoderConfig.Feedback.SensorToMechanismRatio = 1.0;
    wristMotor_CANcoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Get signal and sensor objects for CANcoder
    canCoderMagnetHealth = canCoder.getMagnetHealth();
    canCoderPosition = canCoder.getPosition();

    // Create CANcoder configuration
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;          // TODO verify cancoder direction
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = WristConstants.cancoderDiscontinuityPoint;

    // Apply the configurations to the CANcoder.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    canCoderConfigurator.apply(canCoderConfig);
    

    // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
    // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
    // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
    // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
    // above should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(250);

    // Calibrate if the CANcoder is connected
    usingCANcoder = isCANcoderConnected();
    if (usingCANcoder) {
      encoderZero = WristConstants.offsetAngleCANcoder;
      wristCalibrated = true;
      lastCalibrationState = wristCalibrated;
      DataLogUtil.writeLogEcho(true, "Wrist", "CANcoder calibrated", "Enc Zero", encoderZero,  "CANcoder Rot", getCANcoderRotationsRaw(), 
          "Enc Raw", getWristEncoderRotationsRaw(), "Angle", getWristAngle(), "Target", getCurrentWristTarget());

      // Set software limits after setting encoderZero
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.UPPER_LIMIT.value);
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.LOWER_LIMIT.value);
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations to the motor.
      // Use the CanCoder, since it is reading properly
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      wristMotorConfigurator.apply(wristMotor_CANcoderConfig);

      // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
      // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
      // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
      // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
      // above should take care of this, but it does not always wait long enough.
      // So, add a wait time here:
      Wait.waitTime(250);
    } else {
      wristCalibrated = false;
      lastCalibrationState = wristCalibrated;
      // Record the missing CanCoder as a sticky fault
      RobotPreferences.recordStickyFaults("Wrist-CANcoder");
      DataLogUtil.writeLogEcho(true, "Wrist", "CANcoder not calibrated");
      
      // Apply the configurations to the motor.
      // Use the built-in encoder, since the CanCoder isn't reading properly.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      wristMotorConfigurator.apply(wristMotor_RotorEncoderConfig);

      // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
      // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
      // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
      // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
      // above should take care of this, but it does not always wait long enough.
      // So, add a wait time here:
      Wait.waitTime(250);
    }

    stopWrist();
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  // ********** Wrist movement methods

  /**
   * Stops the wrist motor.
   */
  public void stopWrist() {
    setWristPercentOutput(0.0);
  }

  /**
   * Sets the percent output of the wrist motor using voltage compensation.
   * <b>NOTE: There are no elevator interlocks on this method!
   * @param percent output percent, -1.0 to 1.0 (positive = up, negative = down)
   */
  public void setWristPercentOutput(double percent) {
    if (wristCalibrated) {
      percent = MathUtil.clamp(percent, -WristConstants.maxPercentOutput, WristConstants.maxPercentOutput);
    } else {
      percent = MathUtil.clamp(percent, -WristConstants.maxUncalibratedPercentOutput, WristConstants.maxUncalibratedPercentOutput);
    }

    // TODO add interlocks with elevator, algaeGrabber, and coralEffector

    wristMotor.setControl(wristVoltageControl.withOutput(percent * WristConstants.compensationVoltage));
  }

  /**
   * Gets the percent output of the wrist.
   * @return percent output percent, -1.0 to 1.0 (positive = up, negative = down)
   */
  public double getWristMotorPercentOutput() {
    wristDutyCycle.refresh(); // Verified that this is not a blocking call
    return wristDutyCycle.getValueAsDouble();
  }

  /**
   * Gets whether the wrist is in position control or direct percent output control.
   * @return true = position control, false = direct percent output control
   */
  public boolean isWristMotorPositionControl() {
    return (wristControlMode.refresh().getValue() == ControlModeValue.PositionVoltage) || 
            (wristControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltage) ||
            (wristControlMode.refresh().getValue() == ControlModeValue.PositionVoltageFOC) || 
            (wristControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltageFOC);
  }

  /**
   * Only works when encoder is working and calibrated
   * Interlocks with elevator position
   * @param angle target angle, per WristContstants.WristAngle
   */
  public void setWristAngle(WristAngle angle) {
    setWristAngle(angle.value);
  }

  /**
   * Sets the angle of the wrist of the encoder is working and calibrated, operating with interlocks.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   */
  public void setWristAngle(double angle) {
    if (wristCalibrated) {
      // Keep the wrist in usable range
      safeAngle = MathUtil.clamp(angle, WristConstants.WristAngle.LOWER_LIMIT.value, WristConstants.WristAngle.UPPER_LIMIT.value);

      // TODO add interlocks with elevator, algaeGrabber, and coralEffector
      
      // Phoenix6 PositionVoltage control:  Position is in rotor rotations, FeedFoward is in Volts
      wristMotor.setControl(wristMMVoltageControl.withPosition(wristDegreesToEncoderRotations(safeAngle))
                            .withFeedForward(WristConstants.kG * Math.cos(safeAngle * Math.PI / 180.0)) );

      DataLogUtil.writeLog(false, subsystemName, "setWristAngle", "Desired Angle", angle, "Set Angle", safeAngle);
      SmartDashboard.putNumber("Wrist set raw rot", wristDegreesToEncoderRotations(safeAngle));
    }  
  }

  /**
   * Gets the angle that the wrist is trying to move to. If the wrist is not calibrated, then this value is 
   * the lowerLimit, since we really do not know where the wrist is at. If the wrist is in manual control mode, 
   * then this value is the actual wrist position.
   * @return desired wrist angle, in degrees
   */
  public double getCurrentWristTarget() {
    double currentTarget;

    if (wristCalibrated) {
      if (isWristMotorPositionControl()) {
        currentTarget = safeAngle;
      } else {
        // If we are not in position control mode, then we are not moving towards a target (and the target
        // angle may be undefined). So, get the actual wrist angle instead.
        currentTarget = getWristAngle();
      }
      return currentTarget;
    } else {
      // Wrist is not calibrated. Assume we are at lowerLimit to engage all interlocks, since we really 
      // do not know where the wrist is at.
      return WristAngle.LOWER_LIMIT.value;
    }
  }

  //****** CANcoder methods

  /**
   * Gets whether the CANcoder is connected to the robot and reading the magnet
   * @return true = CANcoder is connected, false = CANcoder is not connected
   */
  public boolean isCANcoderConnected() {
    canCoderMagnetHealth.refresh();
    MagnetHealthValue magnetHealth = canCoderMagnetHealth.getValue();
    
    return canCoder.isConnected() && 
      ( magnetHealth == MagnetHealthValue.Magnet_Green ||
        magnetHealth == MagnetHealthValue.Magnet_Orange);
  }

  /**
   * Gets the raw number of rotations from the CANcoder.
   * @return raw encoder reading, in CANcoder rotations
   */
  public double getCANcoderRotationsRaw() {
    canCoderPosition.refresh(); // Verified that this is not a blocking call
    return canCoderPosition.getValueAsDouble();
  }

  //****** Kraken encoder methods (may be reading internal encoder or may be reading remote CANcoder)

  /**
   * Gets the raw number of rotations from the Kraken encoder.
   * @return raw encoder reading, in pinion rotations
   */
  public double getWristEncoderRotationsRaw() {
    wristEncoderPosition.refresh(); // Verified that this is not a blocking call
    return wristEncoderPosition.getValueAsDouble();
  }

  /**
   * Gets the current wrist angle from the Kraken encoder, assuming the encoder is calibrated.
   * NOTE: This is for internal subsystem use only. Use getWristAngle() when calling from outside.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoderDegrees() {
    // DO NOT normalize this angle. It should not wrap, since the wrist mechanically can not cross the -180/+180 degree point.
    return getWristEncoderRotationsRaw() * WristConstants.kWristDegreesPerRotation - encoderZero;
  }

  /**
   * Returns the angle that the wrist is currently positioned at. If the wrist is not calibrated, then this value is 
   * the lowerLimit, since we really do not know where the wrist is at.
   * @return current wrist angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   */
  public double getWristAngle() {
    if (wristCalibrated) return getWristEncoderDegrees();
    else return WristAngle.LOWER_LIMIT.value;
  }

  /**
   * Converts the wrist angle from degrees to rotations, assuming that the encoder is calibrated.
   * @param degrees wrist angle, in degrees
   * @return wrist angle, in rotations
   */
  private double wristDegreesToEncoderRotations(double degrees) {
    return (degrees + encoderZero) / WristConstants.kWristDegreesPerRotation;
  }

  // ********** Internal Kraken calibration methods

  /**
   * Gets whether the wrist is calibrated.
   * @return true = wrist is calibrated and working, false = wrist is not calibrated
   */
  public boolean isWristCalibrated() {
    return wristCalibrated;
  }  

  /**
   * Stops the wrist and sets wristCalibrated to false.
   */
  public void setWristUncalibrated() {
    stopWrist();
    wristCalibrated = false;
    lastCalibrationState = wristCalibrated;

    DataLogUtil.writeLogEcho(true, "Wrist", "Uncalibrate", 
      "CANcoder Rot", getCANcoderRotationsRaw(), "Enc Raw", getWristEncoderRotationsRaw(), "Angle", getWristAngle(), "Target", getCurrentWristTarget());
  }

  /**
   * Calibrates the wrist encoder, assuming we know the its current angle, and sets wristCalibrated to true.
   * @param angle current angle the wrist is physically at, in degrees
   */
  public void calibrateWristEncoder(double angle) {
    stopWrist(); // Stop the motor so it does not jump to a new value
    encoderZero = getWristEncoderRotationsRaw() * WristConstants.kWristDegreesPerRotation - angle;
    wristCalibrated = true;
    lastCalibrationState = wristCalibrated;

    DataLogUtil.writeLogEcho(true, "Wrist", "Calibrate", "Using CANcoder", usingCANcoder,
      "Enc Zero", encoderZero,  "CANcoder Rot", getCANcoderRotationsRaw(), "Enc Raw", getWristEncoderRotationsRaw(), "Angle", getWristAngle(), "Target", getCurrentWristTarget());

    if (usingCANcoder) {
      // Set software limits after setting encoderZero
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.UPPER_LIMIT.value);
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.LOWER_LIMIT.value);
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      wristMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      wristMotorConfigurator.apply(wristMotor_CANcoderConfig);      
    } else {
      // Set software limits after setting encoderZero
      wristMotor_RotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.UPPER_LIMIT.value);
      wristMotor_RotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.LOWER_LIMIT.value);
      wristMotor_RotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      wristMotor_RotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      wristMotorConfigurator.apply(wristMotor_RotorEncoderConfig);      
    }
  }

  /**
   * Adjusts the current calibration degrees of the wrist by a small amount.
   * Will not do anything if wrist in uncalibrated.
   * @param deltaDegrees degrees to move (positive = down, negative = up)
   */
  public void nudgeWristAngle(double deltaDegrees) {
    // This is dangerous.  Do not use this code for now.  -Don

    /*
    // Do not attempt to nudge if the wrist is not calibrated
    if (!wristCalibrated) return;

    // Save the current write control method (position vs voltage/off)
    boolean isPositionControl = isWristMotorPositionControl();

    // Adjust by recalibrating with a modified degrees, then set to the new angle
    // Note that calibrating the wrist turns off position control.
    calibrateWristEncoder(getWristEncoderDegrees() + deltaDegrees);

    // Only set the angle if in position control mode
    if (isPositionControl) {
      setWristAngle(safeAngle);
    }
    
    DataLogUtil.writeLogEcho(true, "Wrist", "NudgeWristAngle", 
      "Enc Zero", encoderZero,  "CANcoder Rot", getCANcoderRotationsRaw(), "Enc Raw", getWristEncoderRotationsRaw(), "Angle", getWristAngle(), "Target", getCurrentWristTarget());

    */
  }

  //****** Information methods
  
  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */ 
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Writes information about the wrist to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateWristLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogWristTemp.append(wristTemp.refresh().getValueAsDouble(), timeNow);
      dLogWristPctOut.append(getWristMotorPercentOutput(), timeNow);
      dLogWristCurrent.append(wristStatorCurrent.refresh().getValueAsDouble(), timeNow);
      // dLogWristVoltage.append(wristVoltage.refresh().getValueAsDouble(), timeNow);
      dLogBatteryVoltage.append(wristSupplyVoltage.refresh().getValueAsDouble(), timeNow);
      dLogCANcoderRotRaw.append(getCANcoderRotationsRaw(), timeNow);
      // dLogWristEncoderRotRaw.append(getWristEncoderRotationsRaw(), timeNow);
      dLogWristCalZero.append(encoderZero, timeNow);
      dLogWristTarget.append(getCurrentWristTarget(), timeNow);
      dLogWristEncAngle.append(getWristEncoderDegrees(), timeNow);
      dLogWristAngle.append(getWristAngle(), timeNow);
      dLogWristVelDPS.append(wristEncoderVelocity.refresh().getValueAsDouble() * WristConstants.kWristDegreesPerRotation, timeNow);
      // dLogWristAccelDPS2.append(wristEncoderAcceleration.refresh().getValueAsDouble() * WristConstants.kWristDegreesPerRotation, timeNow);
      dLogWristCalibrated.append(isWristCalibrated(), timeNow);
      dLogCANcoderConnected.append(isCANcoderConnected(), timeNow);
    }
  }

  @Override
  public void periodic() {
    // TODO verify that this works
    if (!isWristCalibrated() && !sent) {
      led.sendEvent(StripEvents.SUBSYSTEM_UNCALIBRATED);
      sent = true;
    }

    // TODO verify that this works
    if (lastCalibrationState != isWristCalibrated()) {
      if (!isWristCalibrated()) led.sendEvent(StripEvents.SUBSYSTEM_UNCALIBRATED);
      else led.sendEvent(StripEvents.NEUTRAL);
      lastCalibrationState = isWristCalibrated();
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Wrist CANcoder connected", isCANcoderConnected());
      SmartDashboard.putBoolean("Wrist calibrated", wristCalibrated);
      SmartDashboard.putBoolean("Wrist Using CANcoder", usingCANcoder);
      SmartDashboard.putNumber("Wrist CANcoder raw", getCANcoderRotationsRaw());
      SmartDashboard.putNumber("Wrist encoder raw", getWristEncoderRotationsRaw());
      SmartDashboard.putNumber("Wrist angle", getWristEncoderDegrees());
      SmartDashboard.putNumber("Wrist target angle", getCurrentWristTarget());
      SmartDashboard.putNumber("Wrist output", getWristMotorPercentOutput());
      SmartDashboard.putNumber("Wrist temp", wristTemp.refresh().getValueAsDouble());
    }

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateWristLog(false);
    }

    // If the CANcoder stops reading, apply RotorEncoder configuration and stop using the CANcoder
    // This condition should only occur one time (if at all)
    if (usingCANcoder && !isCANcoderConnected()) {
      setWristUncalibrated();
      usingCANcoder = false;

      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      wristMotorConfigurator.apply(wristMotor_RotorEncoderConfig);

      RobotPreferences.recordStickyFaults("Wrist-CANcoder");

      DataLogUtil.writeLogEcho(false, "Wrist", "CANcoder Disconnection",
        "Enc Zero", encoderZero, 
        "CANcoder Angle", getCANcoderRotationsRaw(), 
        "Enc Raw", getWristEncoderRotationsRaw(), 
        "Angle", getWristEncoderDegrees(), 
        "Target", getCurrentWristTarget());
    }

    // If the driver station is disabled, then turn off any position control for the wrist motor
    if (DriverStation.isDisabled()) {
      stopWrist();
    }

    // TODO Apply interlocks for manual motion or safeAngle
  }
}
