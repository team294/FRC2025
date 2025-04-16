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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.RobotPreferences;
import frc.robot.utilities.Wait;
import frc.robot.Constants.*;
import frc.robot.Constants.ClimberConstants.ClimberAngle;
import frc.robot.Constants.ClimberConstants.ServoPosition;

public class Climber extends SubsystemBase implements Loggable {
  
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboard

  // Create climber motor
  private final TalonFX climberMotor = new TalonFX(Ports.CANClimber);
  private final TalonFXConfigurator climberMotorConfigurator = climberMotor.getConfigurator();

  // Create climber servo
  private final Servo climberServo = new Servo(Ports.PWMCLimberServo);

  // We create two configs for the climber motor -- one with RemoteCANcoder and one with RotorEncoder.
  // The default config is RemoteCANcoder. If the CANcoder fails, the climber will the
  // use the RotorEncoder config as a fallback.
  private boolean usingCANcoder;                              // true = using CANcoder configuration, false = using RotorEncoder configuration
  private TalonFXConfiguration climberMotor_RotorEncoderConfig;  // Configuration reading the rotor encoder
  private TalonFXConfiguration climberMotor_CANcoderConfig;      // Configuration reading the CANcoder

  private VoltageOut climberVoltageControl = new VoltageOut(0.0).withEnableFOC(true);
  private PositionVoltage climberPositionControl = new PositionVoltage(0.0).withEnableFOC(true);
  private MotionMagicVoltage climberMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  // Create variables for the climber Kraken motor
  private final StatusSignal<Voltage> climberSupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> climberTemp;       // Motor temp, in degrees Celsius
  private final StatusSignal<Double> climberDutyCycle;       // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> climberStatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> climberEncoderPosition;  // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> climberEncoderVelocity;
  private final StatusSignal<Voltage> climberVoltage;
  private final StatusSignal<ControlModeValue> climberControlMode;
  private final StatusSignal<AngularAcceleration> climberEncoderAcceleration;

  // Create CANcoder
  private final CANcoder canCoder = new CANcoder(Ports.CANClimberEncoder);
  private final CANcoderConfigurator canCoderConfigurator = canCoder.getConfigurator();
  private CANcoderConfiguration canCoderConfig;

  // CANcoder signals and sensors
  private final StatusSignal<MagnetHealthValue> canCoderMagnetHealth;
  private final StatusSignal<Angle> canCoderPosition;            // CANcoder position, in CANcoder rotations

  private double encoderZero = 0;   // Reference raw encoder angle for climber encoder (may be motor built-in encoder or remote CANcoder) at 0 degrees.

  private boolean climberCalibrated = false;                // Default to climber being uncalibrated. Calibrate from robot preferences or "Calibrate Climber Zero" button on dashboard.

  private double safeAngle; // Current climber target on position control on the motor (if in position mode)

  private boolean isInCoastMode = false;

  private ServoPosition servoPosition = ServoPosition.UNKNOWN;

  public Climber(String subsystemName) {
    
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects for motor
    climberSupplyVoltage = climberMotor.getSupplyVoltage();
    climberTemp = climberMotor.getDeviceTemp();
    climberDutyCycle = climberMotor.getDutyCycle();
    climberStatorCurrent = climberMotor.getStatorCurrent();
    climberEncoderPosition = climberMotor.getPosition();
    climberEncoderVelocity = climberMotor.getVelocity();
    climberVoltage = climberMotor.getMotorVoltage();
    climberControlMode = climberMotor.getControlMode();
    climberEncoderAcceleration = climberMotor.getAcceleration();

    // Create the motor configuration for using the rotor encoder
    climberMotor_RotorEncoderConfig = new TalonFXConfiguration();
    climberMotor_RotorEncoderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberMotor_RotorEncoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor_RotorEncoderConfig.Voltage.PeakForwardVoltage = ClimberConstants.compensationVoltage;
    climberMotor_RotorEncoderConfig.Voltage.PeakReverseVoltage = -ClimberConstants.compensationVoltage;
    climberMotor_RotorEncoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    climberMotor_RotorEncoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Turn off soft limits until encoder is calibrated
    climberMotor_RotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climberMotor_RotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    climberMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    climberMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    climberMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    climberMotor_RotorEncoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberMotor_RotorEncoderConfig.Slot0.kP = ClimberConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
    climberMotor_RotorEncoderConfig.Slot0.kI = 0.0;
    climberMotor_RotorEncoderConfig.Slot0.kD = 0.0;
    climberMotor_RotorEncoderConfig.Slot0.kS = ClimberConstants.kS;
    climberMotor_RotorEncoderConfig.Slot0.kV = ClimberConstants.kV;
    climberMotor_RotorEncoderConfig.Slot0.kA = 0.0;

    // Configure Magic Motion settings
    climberMotor_RotorEncoderConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMCruiseVelocity;
    climberMotor_RotorEncoderConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMAcceleration;
    climberMotor_RotorEncoderConfig.MotionMagic.MotionMagicJerk = ClimberConstants.MMJerk;

    // Configure encoder to user for feedback
    climberMotor_RotorEncoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    climberMotor_RotorEncoderConfig.Feedback.RotorToSensorRatio = 1.0;
    climberMotor_RotorEncoderConfig.Feedback.SensorToMechanismRatio = ClimberConstants.kClimberGearRatio;
    climberMotor_RotorEncoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Configure PID for PositionVoltage control
    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    climberPositionControl.Slot = 0;
    climberPositionControl.OverrideBrakeDurNeutral = true;
    climberMMVoltageControl.Slot = 0;
    climberMMVoltageControl.OverrideBrakeDurNeutral = true;

    // Create the motor configuration for using the CANcoder
    climberMotor_CANcoderConfig = new TalonFXConfiguration();

    climberMotor_CANcoderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climberMotor_CANcoderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor_CANcoderConfig.Voltage.PeakForwardVoltage = ClimberConstants.compensationVoltage;
    climberMotor_CANcoderConfig.Voltage.PeakReverseVoltage = -ClimberConstants.compensationVoltage;
    climberMotor_CANcoderConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    climberMotor_CANcoderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Turn off soft limits until encoder is calibrated
    climberMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    climberMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    climberMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    climberMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    climberMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    climberMotor_CANcoderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    climberMotor_CANcoderConfig.Slot0.kP = ClimberConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
    climberMotor_CANcoderConfig.Slot0.kI = 0.0;
    climberMotor_CANcoderConfig.Slot0.kD = 0.0;
    climberMotor_CANcoderConfig.Slot0.kS = ClimberConstants.kS;
    climberMotor_CANcoderConfig.Slot0.kV = ClimberConstants.kV;
    climberMotor_CANcoderConfig.Slot0.kA = 0.0;

    // Configure Magic Motion settings
    climberMotor_CANcoderConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MMCruiseVelocity;
    climberMotor_CANcoderConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.MMAcceleration;
    climberMotor_CANcoderConfig.MotionMagic.MotionMagicJerk = ClimberConstants.MMJerk;

    // Configure encoder to user for feedback
    climberMotor_CANcoderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    climberMotor_CANcoderConfig.Feedback.FeedbackRemoteSensorID = Ports.CANClimberEncoder;
    climberMotor_CANcoderConfig.Feedback.RotorToSensorRatio = ClimberConstants.kClimberGearRatio;
    climberMotor_CANcoderConfig.Feedback.SensorToMechanismRatio = 1.0;
    climberMotor_CANcoderConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Configure CANcoder and rotor encoder to be synced
    // climberMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    // climberMotorConfig.Feedback.FeedbackRemoteSensorID = Ports.CANclimberEncoder;
    // climberMotorConfig.Feedback.RotorToSensorRatio = climberConstants.kclimberGearRatio;
    // climberMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Get signal and sensor objects for CanCoder
    canCoderMagnetHealth = canCoder.getMagnetHealth();
    canCoderPosition = canCoder.getPosition();

    // Create CANcoder configuration
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ClimberConstants.cancoderDiscontinuityPoint;

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
      encoderZero = ClimberConstants.offsetAngleCANcoder;
      climberCalibrated = true;
      DataLogUtil.writeLogEcho(true, "Climber", "CANcoder calibrated", "Enc Zero", encoderZero,  "CANcoder Rot", getCANcoderRotationsRaw(), 
          "Enc Raw", getClimberEncoderRotationsRaw(), "Angle", getClimberAngle(), "Target", getCurrentClimberTarget());

      // Set software limits after setting encoderZero
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberDegreesToEncoderRotations(ClimberAngle.UPPER_LIMIT.value);
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = climberDegreesToEncoderRotations(ClimberAngle.LOWER_LIMIT.value);
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations to the motor.
      // Use the CanCoder, since it is reading properly
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      climberMotorConfigurator.apply(climberMotor_CANcoderConfig);

      // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
      // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
      // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
      // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
      // above should take care of this, but it does not always wait long enough.
      // So, add a wait time here:
      Wait.waitTime(250);
    } else {
      climberCalibrated = false;
      // Record the missing CanCoder as a sticky fault
      RobotPreferences.recordStickyFaults("Climber-CANcoder");
      DataLogUtil.writeLogEcho(true, "Climber", "CANcoder not calibrated");
      
      // Apply the configurations to the motor.
      // Use the built-in encoder, since the CanCoder isn't reading properly.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      climberMotorConfigurator.apply(climberMotor_RotorEncoderConfig);

      // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
      // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
      // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
      // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
      // above should take care of this, but it does not always wait long enough.
      // So, add a wait time here:
      Wait.waitTime(250);
    }

    stopClimber();

    // Prime the DataLog to reduce delay when first enabling the robot
    updateClimberLog(true);
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  // ********** Climber movement methods

  /**
   * Stops the climber motor.
   */
  public void stopClimber() {
    setClimberPercentOutput(0.0);
  }

  /**
   * Sets the percent output of the climber motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = up, negative = down)
   */
  public void setClimberPercentOutput(double percent) {
    if (climberCalibrated) {
      percent = MathUtil.clamp(percent, -ClimberConstants.maxPercentOutput, ClimberConstants.maxPercentOutput);
    } else {
      percent = MathUtil.clamp(percent, -ClimberConstants.maxUncalibratedPercentOutput, ClimberConstants.maxUncalibratedPercentOutput);
    }

    // Don't move down unless the ratchet is Disengaged
    if (getRatchetPosition() != ServoPosition.DISENGAGED && percent < 0) {
      percent = 0.0;
    }

    climberMotor.setControl(climberVoltageControl.withOutput(percent * ClimberConstants.compensationVoltage));
  }

  /**
   * Gets the percent output of the climber.
   * @return percent output, -1.0 to 1.0 (positive = up, negative = down)
   */
  public double getClimberMotorPercentOutput() {
    climberDutyCycle.refresh(); // Verified that this is not a blocking call
    return climberDutyCycle.getValueAsDouble();
  }

  /**
   * Gets whether the climber is in position control or direct percent output control.
   * @return true = position control, false = direct percent output control
   */
  public boolean isClimberMotorPositionControl() {
    return (climberControlMode.refresh().getValue() == ControlModeValue.PositionVoltage) || 
            (climberControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltage) ||
            (climberControlMode.refresh().getValue() == ControlModeValue.PositionVoltageFOC) || 
            (climberControlMode.refresh().getValue() == ControlModeValue.MotionMagicVoltageFOC);
  }

  /**
   * Sets the climber angle, only when the encoder is working and calibrated, operating with interlocks.
   * @param angle target angle, per ClimberContstants.ClimberAngle
   */
  public void setClimberAngle(ClimberAngle angle) {
    setClimberAngle(angle.value);
  }

  /**
   * Sets the angle of the climber only when the encoder is working and calibrated, operating with interlocks.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   */
  public void setClimberAngle(double angle) {
    if (climberCalibrated && getRatchetPosition() == ServoPosition.DISENGAGED) {
      // Keep the climber in usable range
      safeAngle = MathUtil.clamp(angle, ClimberConstants.ClimberAngle.LOWER_LIMIT.value, ClimberConstants.ClimberAngle.UPPER_LIMIT.value);
      
      // Phoenix6 PositionVoltage control: Position is in rotor rotations, FeedFoward is in Volts
      climberMotor.setControl(climberMMVoltageControl.withPosition(climberDegreesToEncoderRotations(safeAngle))
                            .withFeedForward(ClimberConstants.kG * Math.cos(safeAngle * Math.PI / 180.0)) );

      DataLogUtil.writeLog(false, subsystemName, "setClimberAngle", "Desired Angle", angle, "Set Angle", safeAngle);
      SmartDashboard.putNumber("Climber set raw rot", climberDegreesToEncoderRotations(safeAngle));
    }  
  }

  /**
   * Gets the angle that the climber is trying to move to. If the climber is not calibrated, then this value is 
   * the lowerLimit, since we really do not know where the climber is at. If the climber is in manual control mode, 
   * then this value is the actual climber position.
   * @return desired climber angle, in degrees
   */
  public double getCurrentClimberTarget() {
    double currentTarget;

    if (climberCalibrated) {
      if (isClimberMotorPositionControl()) {
        currentTarget = safeAngle;
      } else {
        // If we are not in position control mode, then we are not moving towards a target (and the target
        // angle may be undefined). So, get the actual climber angle instead.
        currentTarget = getClimberAngle();
      }
      return currentTarget;
    } else {
      // Climber is not calibrated. Assume we are at lowerLimit to engage all interlocks, since we really 
      // do not know where the climber is at.
      return ClimberAngle.LOWER_LIMIT.value;
    }
  }

  //****** CANcoder methods

  /**
   * Gets whether the CANcoder is connected to the robot and reading the magnet.
   * @return true = CANcoder is connected, false = CANcoder is not connected
   */
  public boolean isCANcoderConnected() {
    canCoderMagnetHealth.refresh();
    MagnetHealthValue magnetHealth = canCoderMagnetHealth.getValue();
    
    return canCoder.isConnected() && 
      (magnetHealth == MagnetHealthValue.Magnet_Green ||
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
  public double getClimberEncoderRotationsRaw() {
    climberEncoderPosition.refresh(); // Verified that this is not a blocking call
    return climberEncoderPosition.getValueAsDouble();
  }

  /**
   * Gets the current climber angle from the Kraken encoder, assuming the encoder is calibrated.
   * NOTE: This is for internal subsystem use only. Use getClimberAngle() when calling from outside.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getClimberEncoderDegrees() {
    // DO NOT normalize this angle. It should not wrap, since the climber mechanically can not cross the -180/+180 degree point.
    return getClimberEncoderRotationsRaw() * ClimberConstants.kClimberDegreesPerRotation - encoderZero;
  }

  /**
   * Returns the angle that the climber is currently positioned at. If the climber is not calibrated, then this value is 
   * the lowerLimit, since we really do not know where the climber is at.
   * @return current climber angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   */
  public double getClimberAngle() {
    if (climberCalibrated) return getClimberEncoderDegrees();
    else return ClimberAngle.LOWER_LIMIT.value;
  }

  /**
   * Converts the climber angle from degrees to rotations, assuming that the encoder is calibrated.
   * @param degrees climber angle, in degrees
   * @return climber angle, in rotations
   */
  private double climberDegreesToEncoderRotations(double degrees) {
    return (degrees + encoderZero) / ClimberConstants.kClimberDegreesPerRotation;
  }

  // ********** Internal Kraken calibration methods

  /**
   * Gets whether the encoder is calibrated.
   * @return true = encoder is calibrated and working, false = encoder is not calibrated
   */
  public boolean isEncoderCalibrated() {
    return climberCalibrated;
  }  

  /**
   * Stops the climber and sets climberCalibrated to false.
   */
  public void setClimberUncalibrated() {
    stopClimber();
    climberCalibrated = false;

    DataLogUtil.writeLogEcho(true, "Climber", "Uncalibrate", 
      "CANcoder Rot", getCANcoderRotationsRaw(), "Enc Raw", getClimberEncoderRotationsRaw(), "Angle", getClimberAngle(), "Target", getCurrentClimberTarget());
  }

  /**
   * Calibrates the climber encoder, assuming we know the its current angle, and sets climberCalibrated to true.
   * @param angle current angle the climber is physically at, in degrees
   */
  public void calibrateClimberEncoder(double angle) {
    stopClimber(); // Stop the motor so it does not jump to a new value
    encoderZero = getClimberEncoderRotationsRaw() * ClimberConstants.kClimberDegreesPerRotation - angle;
    climberCalibrated = true;

    DataLogUtil.writeLogEcho(true, "Climber", "Calibrate", "Using CANcoder", usingCANcoder,
      "Enc Zero", encoderZero, "CANcoder Rot", getCANcoderRotationsRaw(), "Enc Raw", getClimberEncoderRotationsRaw(), "Angle", getClimberAngle(), "Target", getCurrentClimberTarget());

    if (usingCANcoder) {
      // Set software limits after setting encoderZero
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberDegreesToEncoderRotations(ClimberAngle.UPPER_LIMIT.value);
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = climberDegreesToEncoderRotations(ClimberAngle.LOWER_LIMIT.value);
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      climberMotor_CANcoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      climberMotorConfigurator.apply(climberMotor_CANcoderConfig);      
    } else {
      // Set software limits after setting encoderZero
      climberMotor_RotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = climberDegreesToEncoderRotations(ClimberAngle.UPPER_LIMIT.value);
      climberMotor_RotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = climberDegreesToEncoderRotations(ClimberAngle.LOWER_LIMIT.value);
      climberMotor_RotorEncoderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      climberMotor_RotorEncoderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  
      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      climberMotorConfigurator.apply(climberMotor_RotorEncoderConfig);      
    }
  }

  /**
   * Set Coast Mode of Climber Motor
   * @param coastMode true = coast, false = brake
   */
  public void setCoastMode(boolean coastMode) {
    if (isInCoastMode != coastMode) {
      climberMotor.setNeutralMode(coastMode ? NeutralModeValue.Coast : NeutralModeValue.Brake);
      isInCoastMode = coastMode;
    }
  }

  /**
   * Returns status of coast mode for climber motor
   * @return true = coast mode, false = brake mode
   */
  public boolean getCoastMode() {
    return isInCoastMode;
  }

  //****** Servo methods

  /**
   * Sets whether the servo is engaged. Engaged means we are not to move the climber down.
   * @param engaged true = engaged, false = disengaged
   */
  public void setRatchetEngaged(boolean engaged) {
    climberServo.set(engaged ? ServoPosition.ENGAGED.value : ServoPosition.DISENGAGED.value);
  }

  /**
   * Current status of the ratchet
   * @return UNKNOWN (unknown or moving), ENGAGED, or DISENGAGED
   */
  public ServoPosition getRatchetPosition() {
    return servoPosition;
  }

  /**
   * Set tracking variable for servo position
   * @param position UNKNOWN (unknown or moving), ENGAGED, or DISENGAGED
   */
  public void setRatchetPositionVariable(ServoPosition position) {
    servoPosition = position;
  }

  /**
   * Set servo position
   * @param position [0.0, 1.0]
   */
  public void setRatchetPosition(double position) {
    position = MathUtil.clamp(position, 0.0, 1.0);
    climberServo.set(position);
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
   * Writes information about the climber to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateClimberLog(boolean logWhenDisabled) {
    DataLogUtil.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "Climber Temp", climberTemp.refresh().getValueAsDouble(),
      "Climber PctOut", getClimberMotorPercentOutput(),
      "Climber Current (Amps)", climberStatorCurrent.refresh().getValueAsDouble(),
      "Climber Voltage", climberVoltage.refresh().getValueAsDouble(),
      "Battery Voltage", climberSupplyVoltage.refresh().getValueAsDouble(),
      "Cancoder Rot Raw", getCANcoderRotationsRaw(),
      "Enc Rot Raw", getClimberEncoderRotationsRaw(),
      "ClimberCalZero", encoderZero,
      "Climber target", getCurrentClimberTarget(),
      "Climber EncAngle (Deg)", getClimberEncoderDegrees(),
      "Climber Angle (Deg)", getClimberAngle(),
      "Climber Vel dps", climberEncoderVelocity.refresh().getValueAsDouble() * ClimberConstants.kClimberDegreesPerRotation,
      "Climber Accel dps2", climberEncoderAcceleration.refresh().getValueAsDouble() * ClimberConstants.kClimberDegreesPerRotation,
      "CANcoder Connected", isCANcoderConnected()
    );
  }

  @Override
  public void periodic() {
    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Climber CANcoder connected", isCANcoderConnected());
      SmartDashboard.putBoolean("Climber calibrated", climberCalibrated);
      SmartDashboard.putBoolean("Climber Using CANcoder", usingCANcoder);
      SmartDashboard.putBoolean("Climber Ratchet Disengaged", getRatchetPosition() == ServoPosition.DISENGAGED); // Disengaged so red on dashboard = don't move climber down
      SmartDashboard.putString("Climber Ratchet Position", getRatchetPosition().toString());
      SmartDashboard.putNumber("Climber CANcoder raw", getCANcoderRotationsRaw());
      SmartDashboard.putNumber("Climber encoder raw", getClimberEncoderRotationsRaw());
      SmartDashboard.putNumber("Climber angle", getClimberEncoderDegrees());
      SmartDashboard.putNumber("Climber target angle", getCurrentClimberTarget());
      SmartDashboard.putNumber("Climber output", getClimberMotorPercentOutput());
      SmartDashboard.putNumber("Climber temp", climberTemp.refresh().getValueAsDouble());
    }

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateClimberLog(false);
    }

    // If the CANcoder stops reading, apply RotorEncoder configuration and stop using the CANcoder
    // This condition should only occur one time (if at all)
    if (usingCANcoder && !isCANcoderConnected()) {
      setClimberUncalibrated();
      usingCANcoder = false;

      // Apply the configurations to the motor.
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
      climberMotorConfigurator.apply(climberMotor_RotorEncoderConfig);

      RobotPreferences.recordStickyFaults("Climber-CANcoder");

      DataLogUtil.writeLogEcho(false, "Climber", "CANcoder Disconnection",
        "Enc Zero", encoderZero, 
        "CANcoder Angle", getCANcoderRotationsRaw(), 
        "Enc Raw", getClimberEncoderRotationsRaw(), 
        "Angle", getClimberEncoderDegrees(), 
        "Target", getCurrentClimberTarget());
    }

    // If the driver station is disabled, then turn off any position control for the climber motor
    if (DriverStation.isDisabled()) {
      stopClimber();
    }
  }
}
