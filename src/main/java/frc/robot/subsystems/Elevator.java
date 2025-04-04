// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.LED.StripEvents;
import frc.robot.Constants.Ports;
import frc.robot.utilities.ElevatorProfileGenerator;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.Wait;

public class Elevator extends SubsystemBase implements Loggable {
  
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private final String subsystemName;   // Subsystem name for use in file logging and dashboarrd
  private final LED led;

  // Create variables for the Elevator Kraken motor 1
  private final StatusSignal<Voltage> elevatorMotor1SupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> elevatorMotor1Temp;       // Motor temperature, in degrees Celsius
  private final StatusSignal<Double> elevatorMotor1DutyCycle;       // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> elevatorMotor1StatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> elevatorMotor1EncoderPosition;  // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> elevatorMotor1EncoderVelocity;
  private final StatusSignal<Voltage> elevatorMotor1Voltage;

  // Create variables for the Elevator Kraken motor 2
  private final StatusSignal<Voltage> elevatorMotor2SupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> elevatorMotor2Temp;       // Motor temperature, in degrees Celsius
  private final StatusSignal<Double> elevatorMotor2DutyCycle;       // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> elevatorMotor2StatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> elevatorMotor2EncoderPosition;  // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> elevatorMotor2EncoderVelocity;
  private final StatusSignal<Voltage> elevatorMotor2Voltage;

  private final TalonFX elevatorMotor1 = new TalonFX(Constants.Ports.CANElevator1);
  private final TalonFX elevatorMotor2 = new TalonFX(Ports.CANElevator2);
  private final TalonFXConfigurator elevatorMotor1Configurator = elevatorMotor1.getConfigurator();
  private final TalonFXConfigurator elevatorMotor2Configurator = elevatorMotor2.getConfigurator();
  private TalonFXConfiguration elevatorMotor1Config;
  private TalonFXConfiguration elevatorMotor2Config;
  private VoltageOut elevatorMotor1VoltageControl = new VoltageOut(0.0);
  private VoltageOut elevatorMotor2VoltageControl = new VoltageOut(0.0);

  // Create lower limit and upper limit sensors
  private final DigitalInput lowerLimitSensor1 = new DigitalInput(Ports.DIOElevatorLowerLimitSensor1);
  private final DigitalInput lowerLimitSensor2 = new DigitalInput(Ports.DIOElevatorLowerLimitSensor2);

  // Create the mmotion profile generator
  private final ElevatorProfileGenerator elevatorProfile;

  private boolean elevatorCalibrated = false;       // true = encoder is working and calibrated, false = not calibrated
  private boolean lastCalibrationState = elevatorCalibrated; // Store last calibration state for LEDs
  private boolean elevatorPositionControl = false;  // true = position control mode (motion profile), false = manual control mode (percent output)

  private boolean sent = false; // true = uncal. state sent to LEDs

  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTemp1 = new DoubleLogEntry(log, "/Elevator/Temp1");
  private final DoubleLogEntry dLogPctOut1 = new DoubleLogEntry(log, "/Elevator/PctOut1");
  private final DoubleLogEntry dLogCurrent1 = new DoubleLogEntry(log, "/Elevator/Current1");
  private final DoubleLogEntry dLogBusVolt1 = new DoubleLogEntry(log, "/Elevator/BusVolt1");
  private final DoubleLogEntry dLogEncInch1 = new DoubleLogEntry(log, "/Elevator/EncInch1");
  private final DoubleLogEntry dLogVel1 = new DoubleLogEntry(log, "/Elevator/Velocity1");
  private final DoubleLogEntry dLogTemp2 = new DoubleLogEntry(log, "/Elevator/Temp2");
  private final DoubleLogEntry dLogPctOut2 = new DoubleLogEntry(log, "/Elevator/PctOut2");
  private final DoubleLogEntry dLogCurrent2 = new DoubleLogEntry(log, "/Elevator/Current2");
  private final DoubleLogEntry dLogBusVolt2 = new DoubleLogEntry(log, "/Elevator/BusVolt2");
  private final DoubleLogEntry dLogEncInch2 = new DoubleLogEntry(log, "/Elevator/EncInch2");
  private final DoubleLogEntry dLogVel2 = new DoubleLogEntry(log, "/Elevator/Velocity2");
  private final DoubleLogEntry dLogTargetPos = new DoubleLogEntry(log, "/Elevator/TargetPos");
  private final BooleanLogEntry dLogCalibrated = new BooleanLogEntry(log, "/Elevator/Calibrated");
  private final BooleanLogEntry dLogPosControl = new BooleanLogEntry(log, "/Elevator/PositionControl");
  private final BooleanLogEntry dLogLowerLimit = new BooleanLogEntry(log, "/Elevator/AtLowerLimit");


  /**
   * Creates the elevator subsystem.
   * @param subsystemName name of the subsystem for file logging
   */
  public Elevator(String subsystemName, LED led) {
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.subsystemName = subsystemName;
    this.led = led;

    // Get signal and sensor objects for motor 1
    elevatorMotor1SupplyVoltage = elevatorMotor1.getSupplyVoltage();
    elevatorMotor1Temp = elevatorMotor1.getDeviceTemp();
    elevatorMotor1DutyCycle = elevatorMotor1.getDutyCycle();
    elevatorMotor1StatorCurrent = elevatorMotor1.getStatorCurrent();
    elevatorMotor1EncoderPosition = elevatorMotor1.getPosition();
    elevatorMotor1EncoderVelocity = elevatorMotor1.getVelocity();
    elevatorMotor1Voltage = elevatorMotor1.getMotorVoltage();

    // Get signal and sensor objects for motor 2
    elevatorMotor2SupplyVoltage = elevatorMotor2.getSupplyVoltage();
    elevatorMotor2Temp = elevatorMotor2.getDeviceTemp();
    elevatorMotor2DutyCycle = elevatorMotor2.getDutyCycle();
    elevatorMotor2StatorCurrent = elevatorMotor2.getStatorCurrent();
    elevatorMotor2EncoderPosition = elevatorMotor2.getPosition();
    elevatorMotor2EncoderVelocity = elevatorMotor2.getVelocity();
    elevatorMotor2Voltage = elevatorMotor2.getMotorVoltage();

    // Instantiate the motion profile generator
    elevatorProfile = new ElevatorProfileGenerator(this);

    // Configure the elevator motor 1
    elevatorMotor1Config = new TalonFXConfiguration();
    elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor1Config.Voltage.PeakForwardVoltage = ElevatorConstants.compensationVoltage * ElevatorConstants.maxPercentOutput;
    elevatorMotor1Config.Voltage.PeakReverseVoltage = -ElevatorConstants.compensationVoltage * ElevatorConstants.maxPercentOutput;
    elevatorMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds
    elevatorMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    elevatorMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to motor 1
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply
    elevatorMotor1Configurator.apply(elevatorMotor1Config);

    // Configure the elevator motor 2
    elevatorMotor2Config = new TalonFXConfiguration();
    elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor2Config.Voltage.PeakForwardVoltage = ElevatorConstants.compensationVoltage;
    elevatorMotor2Config.Voltage.PeakReverseVoltage = -ElevatorConstants.compensationVoltage;
    elevatorMotor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds
    elevatorMotor2Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    elevatorMotor2Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to motor 2
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply
    elevatorMotor2Configurator.apply(elevatorMotor2Config);

    // This is a blocking call and will wait up to 200ms for the zero to apply
    checkAndZeroElevatorEncoders();

    // Wait 0.25 seconds before checking the limit switch or encoder ticks.
    // The reason is that zeroing the encoder (above) can be delayed up to 50ms for a round trip 
    // from the Rio to the Kraken and back to the Rio. So, reading position could give the wrong value 
    // if we do not wait (random weird behavior). Verified this is still needed after migrating to Phoenix 6.
    // DO NOT GET RID OF THIS WITHOUT TALKING TO DON.
    Wait.waitTime(250);

    // Start the elevator in uncalibrated mode unless it is properly zeroed
    elevatorCalibrated = isElevatorAtLowerLimit() && (getElevatorEncoderRotations(1) == 0 && getElevatorEncoderRotations(2) == 0);
    lastCalibrationState = elevatorCalibrated;

    // Ensure the elevator starts in manual control mode
    stopElevatorMotors();

    // Prime the DataLog to reduce delay when first enabling the robot
    updateLog(true);
  }

  /**
   * Checks if elevator is in position control.
   * @return true = position control mode (motion profile), false = manual control mode (percent output)
   */
  public boolean isElevatorInPositionControl() {
    return elevatorPositionControl;
  }

  // ************ Elevator movement methods

  /**
   * Stops both elevator motors.
   */
  public void stopElevatorMotors() {
    setElevatorPercentOutput(0);
  }

  /**
   * Sets the percent output of both elevator motors.
   * This method is for internal use only. It does not change manual vs. profile control modes or check interlocks.
   * @param percent output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  private void setElevatorPercentOutputDirect(double percent) {
    // If the elevator is at or below the lower limit, prevent it from going down any further
    if ((isElevatorAtLowerLimit() || (isElevatorCalibrated() && getElevatorPosition() < ElevatorPosition.LOWER_LIMIT.value)) && percent < 0) {
      percent = 0;
    }

    // If the elevator is at or above the upper limit, prevent it from going up any further
    if ((isElevatorCalibrated() && getElevatorPosition() > ElevatorPosition.UPPER_LIMIT.value) && percent > 0) {
      percent = 0;
    }

    elevatorMotor1.setControl(elevatorMotor1VoltageControl.withOutput(percent * ElevatorConstants.compensationVoltage));
    elevatorMotor2.setControl(elevatorMotor2VoltageControl.withOutput(percent * ElevatorConstants.compensationVoltage));
  }
  
  /**
   * Sets elevator to manual control mode with the specified percent output for both motors.
   * Does not move the elevator if interlocks should prevent movement.
   * @param percent output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  public void setElevatorPercentOutput(double percent) {
    elevatorPositionControl = false;
    elevatorProfile.disableProfileControl();

    // Clamp speed depending on calibration
    if (elevatorCalibrated) {
      percent = MathUtil.clamp(percent, -ElevatorConstants.maxPercentOutput, ElevatorConstants.maxPercentOutput);
    } else {
      percent = MathUtil.clamp(percent, -ElevatorConstants.maxUncalibratedPercentOutput, ElevatorConstants.maxUncalibratedPercentOutput);
    }

    // Do not move the elevator if there is a dangerous condition. TODO add interlocks with wrist, algaeGrabber, and coralEffector

    setElevatorPercentOutputDirect(percent);
  }

  /**
   * Gets the percent output of the voltage compensation limit for an elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  public double getElevatorPercentOutput(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1DutyCycle.refresh();
        return elevatorMotor1DutyCycle.getValueAsDouble();
      case 2:
        elevatorMotor2DutyCycle.refresh();
        return elevatorMotor2DutyCycle.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average percent output of the voltage compensation limit for the elevator motors.
   * @return output percent, -1.0 to +1.0 (positive = up, negative = down)
   */
  public double getElevatorPercentOutput() {
    return (getElevatorPercentOutput(1) + getElevatorPercentOutput(2)) / 2.0;
  }

  /**
   * Sets the target position for the elevator, using motion profile movement.
   * If the elevator is not calibrated, the target position will not change.
   * @param position target, in inches (use ElevatorConstants.ElevatorPosition)
   */
  public void setElevatorProfileTarget(double position) {    
    // TODO add interlocks with wrist, algaeGrabber, and coralEffector
    if (isElevatorCalibrated()) {
      elevatorPositionControl = true;
      position = MathUtil.clamp(position, ElevatorPosition.LOWER_LIMIT.value, ElevatorPosition.UPPER_LIMIT.value);
      elevatorProfile.setProfileTarget(position);
      DataLogUtil.writeMessage(subsystemName, ": setProfileTarget Allowed, Target =", position);
    } else {
      DataLogUtil.writeMessage(subsystemName, ": setProfileTarget Not-Allowed, Target =", position);
    }
  }

  /**
   * Get the target position that the elevator is trying to move to.
   * If the elevator is in manual control mode, returns the actual position.
   * If the elevator is not calibrated, returns +10 inches from the lower limit.
   * @return target, in inches
   */
  public double getCurrentElevatorTarget() {
    if (!isElevatorCalibrated()) {
      return ElevatorConstants.ElevatorPosition.LOWER_LIMIT.value + 10.0;
    } else if (isElevatorInPositionControl()) {
      return elevatorProfile.getFinalPosition();
    } else {
      return getElevatorPosition();
    }
  }

  // ************ Encoder methods

  /**
   * Checks if the elevator is at the lower limit switch.
   * If it is, then zeros the elevator encoder and waits up to 200ms for the new setting to be applied.
   */
  public void checkAndZeroElevatorEncoders() {
    if (isElevatorAtLowerLimit()) {
      stopElevatorMotors(); // Ensure PID loop or motion profile will not move to last set position when resetting encoder

      elevatorMotor1.setPosition(ElevatorPosition.LOWER_LIMIT.value);
      elevatorMotor2.setPosition(ElevatorPosition.LOWER_LIMIT.value);
      elevatorCalibrated = true;
  
      // Set software limits after setting encoderZero
      elevatorMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorPosition.UPPER_LIMIT.value / ElevatorConstants.kElevEncoderInchesPerTick;
      elevatorMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorPosition.LOWER_LIMIT.value / ElevatorConstants.kElevEncoderInchesPerTick;
      elevatorMotor1Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      elevatorMotor1Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // Apply the configurations to motor 1
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply
      elevatorMotor1Configurator.apply(elevatorMotor1Config);

      // Set software limits after setting encoderZero
      elevatorMotor2Config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorPosition.UPPER_LIMIT.value / ElevatorConstants.kElevEncoderInchesPerTick;
      elevatorMotor2Config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorPosition.LOWER_LIMIT.value / ElevatorConstants.kElevEncoderInchesPerTick;
      elevatorMotor2Config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      elevatorMotor2Config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // Apply the configurations to motor 2
      // This is a blocking call and will wait up to 50ms-70ms for the config to apply
      elevatorMotor2Configurator.apply(elevatorMotor2Config);

      DataLogUtil.writeMessageEcho(subsystemName, ":  checkAndZeroElevatorEncoders... Calibrated!");
    }
  }

  /**
   * Checks if both encoders are calibrated.
   * @return true = both calibrated, false = one or both are not calibrated
   */
  public boolean isElevatorCalibrated() {
    return elevatorCalibrated;
  }

  /**
   * Gets the raw encoder rotations of a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return raw encoder reading, in pinion rotations (positive = up, negative = down)
   */
  public double getElevatorEncoderRotations(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1EncoderPosition.refresh();
        return elevatorMotor1EncoderPosition.getValueAsDouble();
      case 2:
        elevatorMotor2EncoderPosition.refresh();
        return elevatorMotor2EncoderPosition.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the vertical position of a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return position from the bottom of the elevator, or the lower limit if uncalibrated, in inches
   */
  public double getElevatorPosition(int motor) {
    if (!elevatorCalibrated) return ElevatorPosition.LOWER_LIMIT.value;

    switch (motor) {
      case 1:
        return getElevatorEncoderRotations(1) * ElevatorConstants.kElevEncoderInchesPerTick;
      case 2:
        return getElevatorEncoderRotations(2) * ElevatorConstants.kElevEncoderInchesPerTick;
      default:
        return ElevatorPosition.LOWER_LIMIT.value;
    }
  }

  /**
   * Gets the vertical position of the elevator using the average position of the motors.
   * @return position from the bottom of the elevator, or the lower limit if uncalibrated, in inches
   */
  public double getElevatorPosition() {
    return (getElevatorPosition(1) + getElevatorPosition(2)) / 2.0;
  }

  /**
   * Gets the velocity of a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return velocity, in in/s (positive = up, negative = down)
   */
  public double getElevatorVelocity(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1EncoderVelocity.refresh();
        return elevatorMotor1EncoderVelocity.getValueAsDouble() * ElevatorConstants.kElevEncoderInchesPerTick;
      case 2:
        elevatorMotor2EncoderVelocity.refresh();
        return elevatorMotor2EncoderVelocity.getValueAsDouble() * ElevatorConstants.kElevEncoderInchesPerTick;
      default:
        return -9999;
    }
  }

  /**
   * Gets the velocity of the elevator using the average of the motors.
   * @return velocity, in in/s (positive = up, negative = down)
   */
  public double getElevatorVelocity() {
    return (getElevatorVelocity(1) + getElevatorVelocity(2)) / 2.0;
  }

  // ************ Sensor methods

  /**
   * Gets whether the elevator is at the lower limit.
   * @return true = at lower limit, false = not at lower limit
   */
  public boolean isElevatorAtLowerLimit() {
    return !lowerLimitSensor1.get() || !lowerLimitSensor2.get();
  }

  /**
   * Gets whether the elevator is at the lower limit.
   * @return true = at lower limit, false = not at lower limit
   */
  public boolean isElevatorAtLowerLimit(int sensor) {
    if (sensor == 1) return !lowerLimitSensor1.get();
    else return !lowerLimitSensor2.get();
  }

  // ************ Information methods
  
  /**
   * Gets the temperature of a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return temperature, in degrees Celsius
   */
  public double getMotorTemp(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1Temp.refresh();
        return elevatorMotor1Temp.getValueAsDouble();
      case 2:
        elevatorMotor2Temp.refresh();
        return elevatorMotor2Temp.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the voltage for a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return voltage, in volts
   */
  public double getElevatorVoltage(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1Voltage.refresh();
        return elevatorMotor1Voltage.getValueAsDouble();
      case 2:
        elevatorMotor2Voltage.refresh();
        return elevatorMotor2Voltage.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average voltage of the elevator motors.
   * @return voltage, in volts
   */
  public double getElevatorVoltage() {
    return (getElevatorVoltage(1) + getElevatorVoltage(2)) / 2.0;
  }

  /**
   * Gets the current that is being drawn by the windings of a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return current, in amps
   */
  public double getStatorCurrent(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1StatorCurrent.refresh();
        return elevatorMotor1StatorCurrent.getValueAsDouble();
      case 2:
        elevatorMotor2StatorCurrent.refresh();
        return elevatorMotor2StatorCurrent.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average current that is being drawn by the windings of the two motors.
   * @return current, in amps
   */
  public double getStatorCurrent() {
    return (getStatorCurrent(1) + getStatorCurrent(2)) / 2.0;
  }

  /**
   * Gets the input bus voltage, measured at the input to the motor controller for a specific elevator motor.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return voltage, in volts
   */
  public double getBusVoltage(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1SupplyVoltage.refresh();
        return elevatorMotor1SupplyVoltage.getValueAsDouble();
      case 2:
        elevatorMotor2SupplyVoltage.refresh();
        return elevatorMotor2SupplyVoltage.getValueAsDouble();
      default:
        return -9999;
    }
  }

  /**
   * Gets the average input bus voltage, measured at the input to the motor controllers.
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return voltage, in volts
   */
  public double getBusVoltage() {
    return (getBusVoltage(1) + getBusVoltage(2)) / 2.0;
  }

  // ************ Periodic methods

  /**
   * Write information about the subsystem to the FileLog
   * @param logWhenDisabled true = log when enabled or disabled, false = only log when enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();
      dLogTemp1.append(getMotorTemp(1), timeNow);
      dLogPctOut1.append(getElevatorPercentOutput(1), timeNow);
      dLogCurrent1.append(getStatorCurrent(1), timeNow);
      dLogBusVolt1.append(getBusVoltage(1), timeNow);
      dLogEncInch1.append(getElevatorPosition(1), timeNow);
      dLogVel1.append(getElevatorVelocity(1), timeNow);
      dLogTemp2.append(getMotorTemp(2), timeNow);
      dLogPctOut2.append(getElevatorPercentOutput(2), timeNow);
      dLogCurrent2.append(getStatorCurrent(2), timeNow);
      dLogBusVolt2.append(getBusVoltage(2), timeNow);
      dLogEncInch2.append(getElevatorPosition(2), timeNow);
      dLogVel2.append(getElevatorVelocity(2), timeNow);
      dLogTargetPos.append(getCurrentElevatorTarget(), timeNow);
      dLogCalibrated.append(elevatorCalibrated, timeNow);
      dLogPosControl.append(elevatorPositionControl, timeNow);
      dLogLowerLimit.append(isElevatorAtLowerLimit(), timeNow);
    }
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = every cycle, false = every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  @Override
  public void periodic() {
    // TODO verify that this works
    // if (!isElevatorCalibrated() && !sent) {
    //   led.sendEvent(StripEvents.SUBSYSTEM_UNCALIBRATED);
    //   sent = true;
    // }

    // TODO verify that this works
    // if (lastCalibrationState != isElevatorCalibrated()) {
    //   if (!isElevatorCalibrated()) led.sendEvent(StripEvents.SUBSYSTEM_UNCALIBRATED);
    //   else led.sendEvent(StripEvents.NEUTRAL);
    //   lastCalibrationState = isElevatorCalibrated();
    // }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Elev Calibrated", isElevatorCalibrated());
      SmartDashboard.putBoolean("Elev Pos Control", isElevatorInPositionControl());

      SmartDashboard.putNumber("Elev Pos", getElevatorPosition());
      SmartDashboard.putNumber("Elev Target", getCurrentElevatorTarget());
      SmartDashboard.putNumber("Elev Velocity", getElevatorVelocity());

      SmartDashboard.putBoolean("Elev Lower Limit", isElevatorAtLowerLimit());
      SmartDashboard.putBoolean("Elev Lower Limit 1", isElevatorAtLowerLimit(1));
      SmartDashboard.putBoolean("Elev Lower Limit 2", isElevatorAtLowerLimit(2));

      SmartDashboard.putNumber("Elev Rotations 1", getElevatorEncoderRotations(1));
      SmartDashboard.putNumber("Elev Rotations 2", getElevatorEncoderRotations(2));
    }

    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    // Sets elevator motors to percent output required as determined by motion profile.
    // Only set percent output if the motion profile is enabled.
    // NOTE: If we are using our motion profile control loop, then set the power directly using setElevatorMotorPercentOutputDirect().
    // Do not call setElevatorMotorPercentOutput(), since that will change elevatorPositionControl to false (manual control).
    if (isElevatorInPositionControl()) {
	    setElevatorPercentOutputDirect(elevatorProfile.trackProfilePeriodic());
    }

    // Elevator is not calibrated and is at the lower limit, so update encoder calibration.
    if (!isElevatorCalibrated() && isElevatorAtLowerLimit()) {
      // This is a blocking call and will wait up to 200ms for the zero to apply.
      checkAndZeroElevatorEncoders();
      DataLogUtil.writeMessageEcho(subsystemName, ": Calibrate Encoders in periodic(), Elevator Pos after cal =", getElevatorPosition());
    }

    // If the elevator is at or below the lower limit, prevent it from going down any further.
    if (isElevatorAtLowerLimit() || (isElevatorCalibrated() && getElevatorPosition() < ElevatorPosition.LOWER_LIMIT.value)) {
      // Elevator is calibrated and under position control. If moving down, set the target to the current position.
      if (isElevatorCalibrated() && isElevatorInPositionControl()) {
        if (getCurrentElevatorTarget() < getElevatorPosition()) {
          setElevatorProfileTarget(getElevatorPosition());
        }

      // Elevator is under manual control. If moving down, stop the motors.
      } else if (!isElevatorInPositionControl()) {
        if (getElevatorPercentOutput() < 0) {
          stopElevatorMotors();
        }

      // Elevator is uncalibrated and under position control. Stop the motors.
      } else {
        stopElevatorMotors();
      }
    }

    // If the elevator is at or above the upper limit, prevent it from going up any further.
    if (isElevatorCalibrated() && getElevatorPosition() > ElevatorPosition.UPPER_LIMIT.value) {
      // Elevator is calibrated and under position control. If moving up, set the target to the current position.
      if (isElevatorCalibrated() && isElevatorInPositionControl()) {
        if (getCurrentElevatorTarget() > getElevatorPosition()) {
          setElevatorProfileTarget(getElevatorPosition());
        }

      // Elevator is under direct control. If moving up, stop the motors.
      } else if (!isElevatorInPositionControl()) {
        if (getElevatorPercentOutput() > 0) {
          stopElevatorMotors();
        }
        
      // Elevator is uncalibrated and under position control. Stop the motors.
      } else {
        stopElevatorMotors();
      }
    }
  }
}
