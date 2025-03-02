// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Wait;
import frc.robot.Constants.*;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;

public class Wrist extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboard

  // Create variables for the wrist Kraken motor
  private final StatusSignal<Voltage> wristSupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> wristTemp;       // Motor temp, in degrees Celsius
  private final StatusSignal<Double> wristDutyCycle;       // Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> wristStatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
	private final StatusSignal<Angle> wristEncoderPosition;  // Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> wristEncoderVelocity;
	private final StatusSignal<Voltage> wristVoltage;
  private final StatusSignal<ControlModeValue> wristControlMode;
  private final StatusSignal<AngularAcceleration> wristEncoderAcceleration;
  
  private final TalonFX wristMotor = new TalonFX(Ports.CANWrist);
  private final TalonFXConfigurator wristMotorConfigurator = wristMotor.getConfigurator();

  // We create two configs -- one with RemoteCANcoder and one with RotorEncoder.
  // The default config is RemoteCANcoder. If the CANcoder fails, the wrist will the
  // use the RotorEncoder config as a fallback.
  // TODO ensure that are calibrating for both intially
  private boolean usingCANcoder = false;                      // true = using CANcoder configuration, false = using RotorEncoder configuration
  private TalonFXConfiguration wristRotorEncoderMotorConfig;  // Configuration reading the rotor encoder
  private TalonFXConfiguration wristCANcoderMotorConfig;      // Configuration reading the CANcoder

  private VoltageOut wristVoltageControl = new VoltageOut(0.0).withEnableFOC(false);
  private PositionVoltage wristPositionControl = new PositionVoltage(0.0).withEnableFOC(false);
  private MotionMagicVoltage wristMMVoltageControl = new MotionMagicVoltage(0.0).withEnableFOC(false);

  // Create CANcoder
  private final CANcoder canCoder = new CANcoder(Ports.CANWristEncoder);
  private final CANcoderConfigurator canCoderConfigurator;
  private CANcoderConfiguration canCoderConfig;

  // CANcoder signals and sensors
	private final StatusSignal<Angle> canCoderPosition;            // CANcoder position, in CANcoder rotations
	private final StatusSignal<AngularVelocity> canCoderVelocity;  // CANcoder velocity, in CANcoder rotations/second

  private double encoderZero = 0;   // Reference raw encoder reading for wrist motor encoder at 0 degrees.

  private boolean calibrationStickyFaultReported = false; // true = we have reported a sticky fault for wrist calibration, false = no sticky fault reported
  private boolean wristCalibrated = false;                // Default to wrist being uncalibrated. Calibrate from robot preferences or "Calibrate Wrist Zero" button on dashboard.

  private double safeAngle; // Current wrist target on position control on the motor (if in position mode)

  public Wrist(String subsystemName, FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects
    wristSupplyVoltage = wristMotor.getSupplyVoltage();
    wristTemp = wristMotor.getDeviceTemp();
    wristDutyCycle = wristMotor.getDutyCycle();
    wristStatorCurrent = wristMotor.getStatorCurrent();
    wristEncoderPosition = wristMotor.getPosition();
    wristEncoderVelocity = wristMotor.getVelocity();
    wristVoltage = wristMotor.getMotorVoltage();
    wristControlMode = wristMotor.getControlMode();
    wristEncoderAcceleration = wristMotor.getAcceleration();

    // Create the configuration for using the rotor encoder
    wristRotorEncoderMotorConfig = new TalonFXConfiguration();
    wristRotorEncoderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		wristRotorEncoderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristRotorEncoderMotorConfig.Voltage.PeakForwardVoltage = WristConstants.compensationVoltage;
		wristRotorEncoderMotorConfig.Voltage.PeakReverseVoltage = -WristConstants.compensationVoltage;
    wristRotorEncoderMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    wristRotorEncoderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Create the configuration for using the CANcoder
    wristCANcoderMotorConfig = new TalonFXConfiguration();
    wristCANcoderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		wristCANcoderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristCANcoderMotorConfig.Voltage.PeakForwardVoltage = WristConstants.compensationVoltage;
		wristCANcoderMotorConfig.Voltage.PeakReverseVoltage = -WristConstants.compensationVoltage;
    wristCANcoderMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    wristCANcoderMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // Configure soft limits on motor
    wristRotorEncoderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.upperLimit.value);
    wristRotorEncoderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.lowerLimit.value);
    wristRotorEncoderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristRotorEncoderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristCANcoderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.upperLimit.value);
    wristCANcoderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.lowerLimit.value);
    wristCANcoderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristCANcoderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    wristRotorEncoderMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    wristRotorEncoderMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    wristRotorEncoderMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    wristRotorEncoderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristRotorEncoderMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    wristRotorEncoderMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    wristCANcoderMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    wristCANcoderMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    wristCANcoderMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    wristCANcoderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    wristCANcoderMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristCANcoderMotorConfig.Feedback.FeedbackRemoteSensorID = Ports.CANWristEncoder;
    wristCANcoderMotorConfig.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;
    wristCANcoderMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Configure CANcoder and rotor encoder to be synced
    // wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    // wristMotorConfig.Feedback.FeedbackRemoteSensorID = Ports.CANWristEncoder;
    // wristMotorConfig.Feedback.RotorToSensorRatio = WristConstants.kWristGearRatio;
    // wristMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Configure PID for PositionVoltage control
    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    wristPositionControl.Slot = 0;
    wristPositionControl.OverrideBrakeDurNeutral = true;
    wristMMVoltageControl.Slot = 0;
    wristMMVoltageControl.OverrideBrakeDurNeutral = true;

    wristRotorEncoderMotorConfig.Slot0.kP = WristConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
		wristRotorEncoderMotorConfig.Slot0.kI = 0.0;
		wristRotorEncoderMotorConfig.Slot0.kD = 0.0;
		wristRotorEncoderMotorConfig.Slot0.kS = WristConstants.kS;
		wristRotorEncoderMotorConfig.Slot0.kV = WristConstants.kV;
		wristRotorEncoderMotorConfig.Slot0.kA = 0.0;

    wristCANcoderMotorConfig.Slot0.kP = WristConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
		wristCANcoderMotorConfig.Slot0.kI = 0.0;
		wristCANcoderMotorConfig.Slot0.kD = 0.0;
		wristCANcoderMotorConfig.Slot0.kS = WristConstants.kS;
		wristCANcoderMotorConfig.Slot0.kV = WristConstants.kV;
		wristCANcoderMotorConfig.Slot0.kA = 0.0;

    // Configure Magic Motion settings
		wristRotorEncoderMotorConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMCruiseVelocity;
		wristRotorEncoderMotorConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MMAcceleration;
		wristRotorEncoderMotorConfig.MotionMagic.MotionMagicJerk = WristConstants.MMJerk;
    wristCANcoderMotorConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMCruiseVelocity;
		wristCANcoderMotorConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MMAcceleration;
		wristCANcoderMotorConfig.MotionMagic.MotionMagicJerk = WristConstants.MMJerk;

    canCoderConfigurator = canCoder.getConfigurator();
    canCoderPosition = canCoder.getPosition();
    canCoderVelocity = canCoder.getVelocity();

    // Configure CANcoder
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = WristConstants.offsetAngleCANcoder;

    // 1 makes absolute position unsigned [0, 1); 0.5 makes it signed [-0.5, 0.5), 0 makes it always negative
    // TODO update this value based on the center of the region of unallowed motion
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    wristMotorConfigurator.apply(wristCANcoderMotorConfig);

    // Apply the configurations to the CANcoder.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    canCoderConfigurator.apply(canCoderConfig);

    // Determine if the wrist is initially calibrated
    if (isCANcoderConnected() && (getWristAngle() == WristConstants.offsetAngleCANcoder)) {
      calibrateWristEncoder(WristConstants.offsetAngleCANcoder);
    }

    // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
    // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
    // asynchronously from this code, so sending the updated configuration to the CANcoder/TalonFX and then receiving 
    // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
    // above should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(250);
  }

  // ********** Wrist movement methods

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

    wristMotor.setControl(wristVoltageControl.withOutput(percent*WristConstants.compensationVoltage));
  }

  /**
   * Stops the wrist motor.
   */
  public void stopWristMotor() {
    setWristPercentOutput(0.0);
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
   * Sets the angle of the wrist of the encoder is working and calibrated, operating with interlocks.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   */
  public void setWristAngle(double angle) {
    if (wristCalibrated) {
      // Keep the wrist in usable range
      safeAngle = MathUtil.clamp(angle, WristConstants.WristAngle.lowerLimit.value, WristConstants.WristAngle.upperLimit.value);

      wristMotor.setControl(wristMMVoltageControl.withPosition(wristDegreesToEncoderRotations(safeAngle)).withFeedForward(WristConstants.kG * Math.cos(safeAngle * Math.PI / 180.0)));

      log.writeLog(false, subsystemName, "setWristAngle", "Desired Angle", angle, "Set Angle", safeAngle);
      SmartDashboard.putNumber("Wrist set raw ticks", wristDegreesToEncoderRotations(safeAngle));
    }  
  }

  /**
   * Sets the angle of the wrist of the encoder is working and calibrated, operating with interlocks.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   * @param height height the elevator is currently at, in inches
   * @param hasAlgae whether the AlgaeGrabber is holding an algae
   * @param inReef whether the robot is against the reef
   */
  public void setWristAngle(double angle, double height, boolean hasAlgae, boolean inReef) {
    if (wristCalibrated) {

      // Keep the wrist in usable range
      double lowerLimit = WristConstants.WristAngleLimitsTest.lowerLimit.apply(height);
      double upperLimit = WristConstants.WristAngleLimitsTest.upperLimit.apply(height); 
      if (hasAlgae) {
        if (inReef) {
          lowerLimit = WristConstants.WristAngleLimitsTest.lowerLimit.apply(height); // TODO Find limits for being in the reef with algae
          upperLimit = WristConstants.WristAngleLimitsTest.upperLimit.apply(height); // See Above
        } else {
          lowerLimit = WristConstants.WristAngleLimitsTest.lowerLimitWithAlgae.apply(height);
          upperLimit = WristConstants.WristAngleLimitsTest.upperLimitWithAlgae.apply(height);
        }
      } else {
        if (inReef) {
          lowerLimit = WristConstants.WristAngleLimitsTest.lowerLimitAtReef.apply(height);
          upperLimit = WristConstants.WristAngleLimitsTest.upperLimitAtReef.apply(height);
        }
      }

      safeAngle = MathUtil.clamp(angle, lowerLimit, upperLimit);

      wristMotor.setControl(wristMMVoltageControl.withPosition(wristDegreesToEncoderRotations(safeAngle)).withFeedForward(WristConstants.kG * Math.cos(safeAngle * Math.PI / 180.0)));

      log.writeLog(false, subsystemName, "setWristAngle", "Desired Angle", angle, "Set Angle", safeAngle);
      SmartDashboard.putNumber("Wrist set raw ticks", wristDegreesToEncoderRotations(safeAngle));
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
      return WristAngle.lowerLimit.value;
    }
  }

  // ********** Wrist region methods

  /**
   * Gets the wrist region for a given angle.
   * NOTE: This is for internal subsystem use only. Use getWristRegion() when calling from outside.
   * @param degrees angle, in degrees
	 * @return corresponding wrist region
	 */
	private WristRegion getRegion(double degrees) {
    // TODO determine if this is necessary and/or if we should actually check the region
    return WristRegion.main; 
	}

  /**
   * Gets the wrist region for a given angle.
   * @param degrees angle, in degrees
	 * @return corresponding wrist region
	 */
  public WristRegion getWristRegion() {
    if (!wristCalibrated) return WristRegion.uncalibrated;
    else return getRegion(getWristAngle());
	}
  
  //****** Internal Kraken encoder methods

   /**
	 * Returns the angle that the wrist is currently positioned at. If the wrist is not calibrated, then this value is 
   * the lowerLimit, since we really do not know where the wrist is at.
	 * @return current wrist angle, in degrees
	 */
  public double getWristAngle() {
    if (wristCalibrated) return getWristEncoderDegrees();
    else return WristAngle.lowerLimit.value;
  }

  /**
   * Gets whether the CANcoder is connected to the robot.
   * @return true = CANcoder is connected, false = CANcoder is not connected
   */
  public boolean isCANcoderConnected() {
    return canCoder.isConnected();
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
   * Gets the raw number of rotations from the Kraken encoder.
   * @return raw encoder reading, in pinion rotations
   */
  public double getWristEncoderRotationsRaw() {
    wristEncoderPosition.refresh(); // Verified that this is not a blocking call
    return wristEncoderPosition.getValueAsDouble();
  }

  /**
   * Converts the wrist angle from degrees to rotations, assuming that the encoder is calibrated.
   * @param degrees wrist angle, in degrees
   * @return wrist angle, in rotations
   */
  private double wristDegreesToEncoderRotations(double degrees) {
    return (degrees + encoderZero) / WristConstants.kWristDegreesPerRotation;
  }

  /**
   * Adjusts the current calibration degrees of the wrist by a small amount.
   * Will not do anything if wrist in uncalibrated.
   * @param deltaDegrees number of degrees to move up/down
   */
  public void nudgeWristAngle(double deltaDegrees) {
    // Do not attempt to nudge if the wrist is not calibrated
    if (!wristCalibrated) return;

    if(wristCalibrated) encoderZero += deltaDegrees;

  }

  // ********** Internal Kraken calibration methods

  /**
	 * Gets whether the encoder is calibrated.
	 * @return true = encoder is calibrated and working, false = encoder is not calibrated
	 */
  public boolean isEncoderCalibrated() {
		return wristCalibrated;
  }  

  /**
	 * Stops the wrist and sets wristCalibrated to false.
	 */
  public void setWristUncalibrated() {
    stopWristMotor();
    wristCalibrated = false;

    log.writeLog(false, "Wrist", "Uncalibrate", 
      "CANcoder Angle", getWristEncoderDegrees(), "Enc Raw", getWristEncoderRotationsRaw(), "Angle", getWristAngle(), "Target", getCurrentWristTarget());
  }

  /**
   * Calibrates the wrist encoder, assuming we know the its current angle, and sets wristCalibrated to true.
   * @param angle current angle the wrist is physically at, in degrees
   */
  public void calibrateWristEncoder(double angle) {
    stopWristMotor(); // Stop the motor so it does not jump to a new value
    encoderZero = getWristEncoderDegrees();
    wristCalibrated = true;

    log.writeLog(true, "Wrist", "Calibrate", 
      "Enc Zero", encoderZero, "CANcoder Angle", getWristEncoderDegrees(), "Enc Raw", getWristEncoderRotationsRaw(), "Angle", getWristAngle(), "Target", getCurrentWristTarget());
  }

  /**
   * Checks if the CANcoder's reading, in degrees, is at or below the lower limit.
   * @return true = wrist is at or below lower limit, false = wrist is above lower limit
   */
  public boolean isWristAtLowerLimit() {
    return getWristEncoderDegrees() <= WristAngle.lowerLimit.value;
  }

  /**
   * Checks if the CANcoder's reading, in degrees, is at or above the upper limit.
   * @return true = wrist is at or above upper limit, false = wrist is below upper limit
   */
  public boolean isWristAtUpperLimit() {
    return getWristEncoderDegrees() >= WristAngle.upperLimit.value;
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

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
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "Wrist Temp", wristTemp.refresh().getValueAsDouble(),
      "Wrist PctOut", getWristMotorPercentOutput(),
      "Wrist Current (Amps)", wristStatorCurrent.refresh().getValueAsDouble(),
      "Wrist Voltage", wristVoltage.refresh().getValueAsDouble(),
      "Enc Pos Raw", getWristEncoderRotationsRaw(),
      "Enc Vel Raw", wristEncoderVelocity.refresh().getValueAsDouble(),
      "Enc Accel Raw", wristEncoderAcceleration.refresh().getValueAsDouble(),
      "WristCalZero", encoderZero,
      "Wrist EncAngle (Deg)", getWristEncoderDegrees(),
      "Wrist Angle (Deg)", getWristAngle(),
      "CANcoder Connected", isCANcoderConnected() ? "TRUE" : "FALES",
      "Wrist LowerLimit", isWristAtLowerLimit() ? "TRUE" : "FALSE",
      "Wrist UpperLimit", isWristAtUpperLimit() ? "TRUE" : "FALSE"
    );
  }

  @Override
  public void periodic() {
    if (log.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("CANcoder connected", isCANcoderConnected());
      SmartDashboard.putBoolean("Wrist calibrated", wristCalibrated);
      SmartDashboard.putBoolean("Wrist lower limit", isWristAtLowerLimit());
      SmartDashboard.putBoolean("Wrist upper limit", isWristAtUpperLimit());
      SmartDashboard.putNumber("Wrist CANcoder angle", getWristEncoderDegrees());
      SmartDashboard.putNumber("Wrist angle", getWristEncoderDegrees());
      SmartDashboard.putNumber("Wrist target angle", getCurrentWristTarget());
      SmartDashboard.putNumber("Wrist encoder raw", getWristEncoderRotationsRaw());
      SmartDashboard.putNumber("Wrist output", getWristMotorPercentOutput());
    }

    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateWristLog(false);
    }

    // If the CANcoder stops reading, apply RotorEncoder configuration and stop using the CANcoder
    if (usingCANcoder && wristCalibrated && !isCANcoderConnected()) {
      setWristUncalibrated();
      usingCANcoder = false;
      wristMotorConfigurator.apply(wristRotorEncoderMotorConfig);
      log.writeLog(false, "Wrist", "CANcoder Disconnection",
        "Enc Zero", encoderZero, 
        "CANcoder Angle", getWristEncoderDegrees(), 
        "Enc Raw", getWristEncoderRotationsRaw(), 
        "Angle", getWristAngle(), 
        "Target", getCurrentWristTarget());
    }

  }
}
