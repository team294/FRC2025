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
import frc.robot.utilities.Wait;
import frc.robot.Constants.*;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.WristConstants.WristRegion;

public class Wrist extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboard

  // Createa variables for the wrist Kraken motor
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
  private TalonFXConfiguration wristMotorConfig;
  private VoltageOut wristVoltageControl = new VoltageOut(0.0);

  private PositionVoltage wristPositionControl = new PositionVoltage(0.0);
  private MotionMagicVoltage wristMMVoltageControl = new MotionMagicVoltage(0.0);

  // Create CANcoder
  private final CANcoder canCoder = new CANcoder(Ports.CANWristEncoder);
  private final CANcoderConfigurator canCoderConfigurator;
  private CANcoderConfiguration canCoderConfig;

  // CANCoder signals and sensors
	private final StatusSignal<Angle> canCoderPosition;            // CANCoder position, in CANCoder rotations
	private final StatusSignal<AngularVelocity> canCoderVelocity;  // CANCoder velocity, in CANCoder rotations/second

  // Variables for encoder zeroing
  private double encoderZero = 0;   // Reference raw encoder reading for wrist motor encoder. Calibration sets this to zero.
  private double canCoderZero = 0;  // Reference raw encoder reading for CANCoder. Calibration sets this to the absolute position from RobotPreferences.

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

    // Configure the motor
    wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotorConfig.Voltage.PeakForwardVoltage = WristConstants.voltageCompSaturation;
		wristMotorConfig.Voltage.PeakReverseVoltage = -WristConstants.voltageCompSaturation;
    wristMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;     // Time from 0 to full power, in seconds
    wristMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    wristMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    wristMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    wristMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure encoder
    wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    wristMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    canCoderConfigurator = canCoder.getConfigurator();
    canCoderPosition = canCoder.getPosition();
    canCoderVelocity = canCoder.getVelocity();

    // Configure CANcoder
    canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // 1 makes absolute position unsigned [0, 1); 0.5 makes it signed [-0.5, 0.5), 0 makes it always negative 
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    // Configure PID for PositionVoltage control
    // NOTE: In Phoenix 6, slots are selected in the ControlRequest (ex. PositionVoltage.Slot)
    wristPositionControl.Slot = 0;
    wristPositionControl.OverrideBrakeDurNeutral = true;
    wristMMVoltageControl.Slot = 0;
    wristMMVoltageControl.OverrideBrakeDurNeutral = true;
    wristMotorConfig.Slot0.kP = WristConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
		wristMotorConfig.Slot0.kI = 0.0;
		wristMotorConfig.Slot0.kD = 0.0;
		wristMotorConfig.Slot0.kS = WristConstants.kS;
		wristMotorConfig.Slot0.kV = WristConstants.kV;
		wristMotorConfig.Slot0.kA = 0.0;

    // Configure Magic Motion settings
		wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity = WristConstants.MMCruiseVelocity;
		wristMotorConfig.MotionMagic.MotionMagicAcceleration = WristConstants.MMAcceleration;
		wristMotorConfig.MotionMagic.MotionMagicJerk = WristConstants.MMJerk;

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    wristMotorConfigurator.apply(wristMotorConfig);

    // Apply the configurations to the CANcoder.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    canCoderConfigurator.apply(canCoderConfig);

    // NOTE!!! When the TalonFX encoder settings are changed above, then the next call to getTurningEncoderDegrees() 
    // may contain an old value, not the value based on the updated configuration settings above!!!!  The CANBus runs 
    // asynchronously from this code, so sending the updated configuration to the CANCoder/TalonFX and then receiving 
    // an updated position measurement back may take longer than this code. The timeouts in the configuration code 
    // above should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(250);
  }


  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  // ********** Wrist movement methods

  public void stopWrist() {
    setWristMotorPercentOutput(0.0);
  }

  /**
   * Sets percent power of wrist motor
   * <p><b> There are no elevator interlocks on this method!!!! </b>
   * @param percentPower between -1.0 (down full speed) and 1.0 (up full speed)
   */
  public void setWristMotorPercentOutput(double percentOutput) {
    if (wristCalibrated) {
      percentOutput = MathUtil.clamp(percentOutput, -WristConstants.maxPercentOutput, WristConstants.maxPercentOutput);
    } else {
      percentOutput = MathUtil.clamp(percentOutput, -WristConstants.maxUncalibratedPercentOutput, WristConstants.maxUncalibratedPercentOutput);
    }

    wristMotor.setControl(wristVoltageControl.withOutput(percentOutput*WristConstants.voltageCompSaturation));
  }

  public double getWristMotorPercentOutput() {
		wristDutyCycle.refresh();			// Verified that this is not a blocking call.
		return wristDutyCycle.getValueAsDouble();
	}

  /**
   * Returns a boolean to indicate if the wrist is in position control or direct
   * percent output control.
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
   * @param angle target angle, in degrees (0 = horizontal in front of robot, + = up, - = down)
   */
  public void setWristAngle(double angle) {
    if (wristCalibrated) {
      // Keep wrist in usable range
      safeAngle = MathUtil.clamp(angle, WristConstants.WristAngle.lowerLimit.value, WristConstants.WristAngle.upperLimit.value);

      // WristRegion curRegion = getRegion(getWristAngle());
      // Check and apply interlocks      

      // Phoenix6 PositionVoltage control:  Position is in rotor rotations, FeedFoward is in Volts
      // wristMotor1.setControl(wristPositionControl.withPosition(wristDegreesToEncoderRotations(safeAngle))
      //                       .withFeedForward(kG * Math.cos(safeAngle*Math.PI/180.0) ));
      // Phoenix6 MotionMagicVoltage control:  Position is in rotor rotations, FeedFoward is in Volts
      wristMotor.setControl(wristMMVoltageControl.withPosition(wristDegreesToEncoderRotations(safeAngle))
                            .withFeedForward(WristConstants.kG * Math.cos(safeAngle*Math.PI/180.0) ));

      log.writeLog(false, subsystemName, "Set angle", "Desired angle", angle, "Set angle", safeAngle);

      SmartDashboard.putNumber("Wrist set raw ticks", wristDegreesToEncoderRotations(safeAngle));
    }  
  }

  /**
	 * Returns the angle that wrist is trying to move to in degrees.
	 * If the wrist is not calibrated, then returns wrist lowerLimit,
   * since we really don't know where the wrist is at.  If the wrist is in manual control mode, then
   * returns the actual wrist position.
	 * @return desired degree of wrist angle
	 */
  public double getCurrentWristTarget() {
    double currentTarget;

    if (wristCalibrated) {
      if (isWristMotorPositionControl()) {
        currentTarget = safeAngle;
      } else {
        // If we are not in position control mode, then we aren't moving towards a target (and the target
        // angle may be undefined).  So, get the actual wrist angle instead.
        currentTarget = getWristAngle();
      }
      return currentTarget;
    } else {
      // Wrist is not calibrated.  Assume we are at back angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristAngle.lowerLimit.value;
    }
  }

  // ****** Wrist region methods

  /**
   * For use in the wrist subsystem only.  Use getWristRegion() when calling from outside this class.
	 * <p>Returns the wrist region for a given angle.
   * @param degrees angle in degrees
	 * @return corresponding wrist region
	 */
	private WristRegion getRegion(double degrees) {
    return WristRegion.main; 
	}

    /**
   * For use in the wrist subsystem only.  Use getWristRegion() when calling from outside this class.
	 * <p>Returns the wrist region for a given angle.
   * @param degrees angle in degrees
	 * @return corresponding wrist region
	 */
  public WristRegion getWristRegion() {
    if (!wristCalibrated) {
      return WristRegion.uncalibrated;
    }

    WristRegion curRegion = getRegion(getWristAngle());

    // if (isWristMotorPositionControl()) {
    //   WristRegion targetRegion = getRegion(safeAngle);

    //   if (targetRegion != WristRegion.main) curRegion = WristRegion.back;
    // }
      
    return curRegion;
	}
  
  //****** Internal Falcon encoder methods

   /**
	 * Returns the angle that wrist is currently positioned at in degrees.
	 * If the wrist is not calibrated, then returns wrist lowerLimit,
   * since we really don't know where the wrist is at.
	 * @return current degree of wrist angle
	 */
  public double getWristAngle() {
    if (wristCalibrated) {
      return getWristEncoderDegrees();
    } else {
      // Wrist is not calibrated.  Assume we are at back angle in keepout region to engage all interlocks,
      // since we really don't know where the wrist is at.
      return WristAngle.lowerLimit.value;
    }
  }

  public boolean isCANcoderConnected() {
    return canCoder.isConnected();
  }
  
  /**
   * For use in the wrist subsystem only.  Use getWristAngle() when calling from outside this class.
   * Assumes that the encoder is calibrated.
   * <p>Gets the current wrist angle from the Falcon encoder.
   * @return current encoder ticks (based on zero) converted to degrees
   */
  private double getWristEncoderDegrees() {
    // DO NOT normalize this angle.  It should not wrap, since the wrist mechanically can not cross the -180/+180 deg point
    return getWristEncoderRotationsRaw()* WristConstants.kWristDegreesPerRotation - encoderZero;
  }

  /**
   * 
   * @return raw encoder reading, in pinion rotations, adjusted direction (positive is towards stowed, negative is towards lower hard stop)
   */
  public double getWristEncoderRotationsRaw() {
    wristEncoderPosition.refresh();          // Verified that this is not a blocking call.
    return wristEncoderPosition.getValueAsDouble();
  }

    /**
   * Converts the wrist position in degrees to rotations.  Assumes that the encoder is calibrated.
   * @param degrees wrist position in degrees
   * @return wrist position in rotations
   */
  private double wristDegreesToEncoderRotations(double degrees) {
    return (degrees + encoderZero) / WristConstants.kWristDegreesPerRotation;
  }

  /**
   * Adjust the current calibration degrees of the wrist by a small amount
   * @param deltaDegrees the number of degrees to move up/down
   */
  public void nudgeWristAngle(double deltaDegrees) {
    if (!wristCalibrated) {
      // Do not attempt this if the wrist is not calibrated
      return;
    }
  }

  //***** Internal Falcon Calibration Methods

  /**
	 * returns whether encoder is calibrated or not
	 * @return true if encoder is calibrated and working, false if encoder broke
	 */
  public boolean isEncoderCalibrated() {
		return wristCalibrated;
  }  

  /**
	 * Stops wrist motor and sets wristCalibrated to false
	 */
  public void setWristUncalibrated() {
    stopWrist();

    log.writeLog(false, "Wrist", "Uncalibrate wrist", 
    "CANcoder angle", getWristEncoderDegrees(), "Enc Raw", getWristEncoderRotationsRaw(), "Wrist angle", getWristAngle(), "Wrist target", getCurrentWristTarget());

    wristCalibrated = false;
  }

  /**
   * Calibrates the wrist encoder, assuming we know the wrist's current angle.
   * Sets wristCalibrated = true.
   * @param angle current angle that the wrist is physically at, in degrees
   */
  public void calibrateWristEncoder(double angle) {
    stopWrist(); // Stop the motor so it doesn't jump to a new value

    encoderZero = getWristEncoderDegrees();
    wristCalibrated = true;

    log.writeLog(true, "Wrist", "Calibrate wrist", "zero value", encoderZero, 
		"CANcoder angle", getWristEncoderDegrees(), "Enc Raw", getWristEncoderRotationsRaw(), "Wrist Angle", getWristAngle(), "Wrist Target", getCurrentWristTarget());
  }


  /** Checks if the CANcoder's reading, in degrees, is at or below the lower limit
   * @return true = wrist is at or below lower limit, false = above lower limit
   */
  public boolean isWristAtLowerLimit() {
    return getWristEncoderDegrees() <= WristAngle.lowerLimit.value;
  }

  /** Checks if the CANcoder's reading, in degrees, is at or above the upper limit
   * @return true = wrist is at or above upper limit, false = below upper limit
   */
  public boolean isWristAtUpperLimit() {
    return getWristEncoderDegrees() >= WristAngle.upperLimit.value;
  }

  /**
   * Writes information about the subsystem to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateWristLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
    "Temp", wristTemp.refresh().getValueAsDouble(),
    "Percent Output", getWristMotorPercentOutput(),
    "Amps", wristStatorCurrent.refresh().getValueAsDouble(),
    "Volts", wristVoltage.refresh().getValueAsDouble(),
    "Enc pos raw", getWristEncoderRotationsRaw(),
    "Enc vel raw", wristEncoderVelocity.refresh().getValueAsDouble(),
    "Enc Accel raw", wristEncoderAcceleration.refresh().getValueAsDouble(),
    "WristCalZero", encoderZero,
    "Wrist degrees", getWristEncoderDegrees(),
    "Wrist angle", getWristAngle(),
    "CANcoder connected", isCANcoderConnected(),
    "Lower limit", isWristAtLowerLimit(),
    "Upper limit", isWristAtUpperLimit()
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

  if(fastLogging || log.isMyLogRotation(logRotationKey)){
      updateWristLog(false);
    }
  }
  //** Wrist using CANcoder not a through-bore, so may not need the 2.5s wait, and a bump switch needed for this automatic calibration functionality
  // // Rev Through-Bore Encoder takes a while to boot up.
  //   // After it boots up, it takes up to 40ms sec to settle into an accurate reading.
  //   // Previously, we waited for 5 periodic cycles after the encoder boots up before calibrating.
  //   // Now, just wait 2.5 seconds after the Wrist constructor.  If the Rev encoder is still not
  //   // reading, then just use the hard stop angle.
  //   if (!wristCalibrated && bootTimer.hasElapsed(2.5) && isWristAtLowerLimit()) {
  //     if (isRevEncoderConnected()) {
  //       // Calibrate Rev encoder
  //       log.writeLogEcho(true, subsystemName, "calibrateEncoder pre", "Rev encoder connecected", true,
  //         "Pre Rev angle", getRevEncoderDegrees());

  //       calibrateRevEncoderDegrees(revEncoderOffsetAngleWrist);

  //       // Copy calibration to wrist encoder
  //       // This sets wristCalibrated to true
  //       calibrateWristEnc(getRevEncoderDegrees());

  //       log.writeLogEcho(true, subsystemName, "calibrateEncoder post", "Rev encoder connecected", true,
  //         "Post Rev angle", getRevEncoderDegrees(), "Post wrist angle", getWristAngle());
  //     } else {
  //       // Wrist is at lower limit, but Rev encoder is not working.  Assume wrist is on the hard stop.
  //       calibrateWristEnc(WristAngle.lowerLimit.value);

  //       log.writeLogEcho(true, subsystemName, "calibrateEncoder post", "Rev encoder connecected", false,
  //         "Post wrist angle", getWristAngle());
  //     }

  //     // Configure soft limits on motor
  //     wristMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.upperLimit.value);
  //     wristMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = wristDegreesToEncoderRotations(WristAngle.lowerLimit.value);
  //     wristMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  //     wristMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

  //     // Apply configuration to the wrist motor. 1 and 2 
  //     // This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
  //     wristMotor1Configurator.apply(wristMotor1Config);
  //   }

  //   // If driver station is no longer disabled and wrist is not calibrated, then 
  //   // record a sticky fault (once)
  //   if (!calibrationStickyFaultReported && !wristCalibrated && !DriverStation.isDisabled()) {
  //     calibrationStickyFaultReported = true;
  //     RobotPreferences.recordStickyFaults("Wrist-Not-calibrated-when-enabled", log);
  //     log.writeLogEcho(true, subsystemName, "calibrate Wrist", "Wrist calibrated", false);
  //   }

  //   // If the driver station is disabled, then turn off any position control for the wrist motor
  //   if (DriverStation.isDisabled()) {
  //     stopWrist();
  //   }

  //   // If the wrist hits the bump switch, then stop the wrist from moving down further
  //   if (isWristAtLowerLimit()) {
  //     if (wristCalibrated && isWristMotorPositionControl()) {
  //       // Wrist is calibrated and under position control, so don't let the position go down any further
  //       if (getCurrentWristTarget() < getWristEncoderDegrees()) {
  //         // Current target angle is below current angle.  Reset target to current angle
  //         setWristAngle(getWristEncoderDegrees());
  //       }
  //     } else {
  //       // Wrist is under voltage (direct speed) control.  Stop it if it is moving down
  //       if (getWristMotorPercentOutput()<0) {
  //         stopWrist();
  //       }
  //     }
  //   }

  //   // Un-calibrates the wrist if the angle is outside of bounds.
  //   // Turned off for right now.  It still occasionally false-triggers, even with 20degree tolerances.
  //   // if (getWristAngle() > WristAngle.upperLimit.value + 20.0 || getWristAngle() < WristAngle.lowerLimit.value - 20.0) {
  //   //   setWristUncalibrated();
  //   //   updateWristLog(true);
  //   // }

}
