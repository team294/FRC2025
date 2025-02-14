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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.Ports;
import frc.robot.utilities.ElevatorProfileGenerator;
import frc.robot.utilities.FileLog;

public class Elevator extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;
  private final String subsystemName;
  private final EndEffector endEffector;

  // Create Kraken variables for elevator motor 1
  private final StatusSignal<Voltage> elevatorMotor1SupplyVoltage; // Incoming bus voltage to motor contelevatorMotor, in volts
  private final StatusSignal<Temperature> elevatorMotor1Temp; // Motor temperature, in degC
  private final StatusSignal<Double> elevatorMotor1DutyCycle; // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> elevatorMotor1StatorCurrent; // Motor stator current, in amps (+=fwd, -=rev)
  private final StatusSignal<Angle> elevatorMotor1EncoderPosition; // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> elevatorMotor1EncoderVelocity;
  private final StatusSignal<Voltage> elevatorMotor1Voltage;

  // Create Kraken variables for elevator motor 2
  private final StatusSignal<Voltage> elevatorMotor2SupplyVoltage; // Incoming bus voltage to motor contelevatorMotor, in volts
  private final StatusSignal<Temperature> elevatorMotor2Temp; // Motor temperature, in degC
  private final StatusSignal<Double> elevatorMotor2DutyCycle; // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> elevatorMotor2StatorCurrent; // Motor stator current, in amps (+=fwd, -=rev)
  private final StatusSignal<Angle> elevatorMotor2EncoderPosition; // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> elevatorMotor2EncoderVelocity;
  private final StatusSignal<Voltage> elevatorMotor2Voltage;

  // Create Krakens for elevator motors
  private final TalonFX elevatorMotor1 = new TalonFX(Constants.Ports.CANElevator1);
  private final TalonFX elevatorMotor2 = new TalonFX(Ports.CANElevator2);
  private final TalonFXConfigurator elevatorMotor1Configurator = elevatorMotor1.getConfigurator();
  private final TalonFXConfigurator elevatorMotor2Configurator = elevatorMotor2.getConfigurator();
  private TalonFXConfiguration elevatorMotor1Config;
  private TalonFXConfiguration elevatorMotor2Config;
  private VoltageOut elevatorMotor1VoltageControl = new VoltageOut(0.0);
  private VoltageOut elevatorMotor2VoltageControl = new VoltageOut(0.0);

  // Create lowerLimit and upperLimit sensors for the elevator
  private final DigitalInput lowerLimitSensor = new DigitalInput(Ports.DIOElevatorLowerLimitSensor);
  private final DigitalInput upperLimitSensor = new DigitalInput(Ports.DIOElevatorUpperLimitSensor);

  private final ElevatorProfileGenerator elevatorProfile;

  private boolean elevatorCalibrated = false; // true = encoder is working and calibrated, false = not calibrated
  private boolean elevatorPositionControl = false; // true = in position control mode (motion profile), false = manual motor control (percent output)

  public Elevator(EndEffector endEffector, String subsystemName, FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    this.subsystemName = subsystemName;
    this.endEffector = endEffector;

    // Get signal and sensor objects
    elevatorMotor1SupplyVoltage = elevatorMotor1.getSupplyVoltage();
    elevatorMotor1Temp = elevatorMotor1.getDeviceTemp();
    elevatorMotor1DutyCycle = elevatorMotor1.getDutyCycle();
    elevatorMotor1StatorCurrent = elevatorMotor1.getStatorCurrent();
    elevatorMotor1EncoderPosition = elevatorMotor1.getPosition();
    elevatorMotor1EncoderVelocity = elevatorMotor1.getVelocity();
    elevatorMotor1Voltage = elevatorMotor1.getMotorVoltage();

    // Get signal and sensor objects
    elevatorMotor2SupplyVoltage = elevatorMotor2.getSupplyVoltage();
    elevatorMotor2Temp = elevatorMotor2.getDeviceTemp();
    elevatorMotor2DutyCycle = elevatorMotor2.getDutyCycle();
    elevatorMotor2StatorCurrent = elevatorMotor2.getStatorCurrent();
    elevatorMotor2EncoderPosition = elevatorMotor2.getPosition();
    elevatorMotor2EncoderVelocity = elevatorMotor2.getVelocity();
    elevatorMotor2Voltage = elevatorMotor2.getMotorVoltage();

    elevatorProfile = new ElevatorProfileGenerator(this, log);

    // Configure the elevator motor 1
    elevatorMotor1Config = new TalonFXConfiguration(); // Factory default configuration
    elevatorMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor1Config.Voltage.PeakForwardVoltage = ElevatorConstants.compensationVoltage;
    elevatorMotor1Config.Voltage.PeakReverseVoltage = -ElevatorConstants.compensationVoltage;
    elevatorMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // Seconds from 0 to full power

    // Supply current limit is typically used to prevent breakers from tripping
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLimit = 60.0; // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // (amps) Lower limit for the current
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLowerTime = 0.2; // (sec) Threshold time
    elevatorMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set elevator motor 1 configuration
    elevatorMotor1Configurator.apply(elevatorMotor1Config);

    // Configure the elevator motor 2
    elevatorMotor2Config = new TalonFXConfiguration(); // Factory default configuration
    elevatorMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor2Config.Voltage.PeakForwardVoltage = ElevatorConstants.compensationVoltage;
    elevatorMotor2Config.Voltage.PeakReverseVoltage = -ElevatorConstants.compensationVoltage;
    elevatorMotor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // Seconds from 0 to full power

    // Supply current limit is typically used to prevent breakers from tripping
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLimit = 60.0; // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // (amps) Lower limit for the current
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLowerTime = 0.2; // (sec) Threshold time
    elevatorMotor2Config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set elevator motor 2 configuration
    elevatorMotor2Configurator.apply(elevatorMotor2Config);

    checkAndZeroElevatorEncoders();
    elevatorCalibrated = isElevatorAtLowerLimit() && (getElevatorEncoderTicks(1) == 0 && getElevatorEncoderTicks(2) == 0);

    stopElevatorMotors();
  }

  /**
   * Check if both encoders are calibrated
   * @return true = calibrated, false = not calibrated
   */
  public boolean isElevatorCalibrated() {
    return elevatorCalibrated;
  }

  /**
   * Check if elevator is in position control
   * @return true = position control, false = direct control
   */
  public boolean isElevatorInPositionControl() {
    return elevatorPositionControl;
  }

  /**
   * Read whether the elevator is at the upper limit
   * @return true = at upper limit, false = not at upper limit
   */
  public boolean isElevatorAtUpperLimit() {
    return !upperLimitSensor.get(); 
  }

  /**
   * Read whether the elevator is at the lower limit
   * @return true = at lower limit, false = not at lower limit
   */
  public boolean isElevatorAtLowerLimit() {
    return !lowerLimitSensor.get();
  }

  /**
   * Set the percent output of both elevator motors
   * @param percent double -1.0 to +1.0 (positive = up, negative = down)
   */
  public void setElevatorPercentOutput(double percent) {
    if (endEffector.isPiecePresentInEntry()) return;
    elevatorPositionControl = false;
    elevatorProfile.disableProfileControl();
    elevatorMotor1.setControl(elevatorMotor1VoltageControl.withOutput(percent * ElevatorConstants.compensationVoltage));
    elevatorMotor2.setControl(elevatorMotor2VoltageControl.withOutput(percent * ElevatorConstants.compensationVoltage));
  }

  /**
   * Set the percent output of both elevator motors without disabling position control
   * SHOULD ONLY BE USED FOR CONTROLLING MOTION PROFILE
   * @param percent double -1.0 to +1.0 (positive = up, negative = down)
   */
  private void setElevatorPercentOutputDirect(double percent) {
    if (endEffector.isPiecePresentInEntry()) return;
    elevatorMotor1.setControl(elevatorMotor1VoltageControl.withOutput(percent * ElevatorConstants.compensationVoltage));
    elevatorMotor2.setControl(elevatorMotor2VoltageControl.withOutput(percent * ElevatorConstants.compensationVoltage));
	}
  
  /**
   * Get the percent output of the voltage compensation limit for an elevator motor
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
	 * @return double -1.0 to +1.0 (positive = up, negative = down)
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
   * Get the average percent output of the voltage compensation limit for the elevator motors
	 * @return double -1.0 to +1.0 (positive = up, negative = down)
	 */
  public double getElevatorPercentOutput() {
    return (getElevatorPercentOutput(1) + getElevatorPercentOutput(2)) / 2;
	}

  /**
   * Stop both elevator motors
   */
  public void stopElevatorMotors() {
    setElevatorPercentOutput(0);
  }

  /**
   * Set the target position for the elevator, using motion profile movement
   * If the elevator is not calibrated, the target position will not change
   * @param position target in inches (use ElevatorConstants.ElevatorPosition)
   */
  public void setElevatorProfileTarget(double position) {
    if (endEffector.isPiecePresentInEntry()) return;
    if (isElevatorCalibrated()) {
      elevatorPositionControl = true;
      position = MathUtil.clamp(position, ElevatorPosition.LOWER_LIMIT.value, ElevatorPosition.UPPER_LIMIT.value);
      elevatorProfile.setProfileTarget(position);
      log.writeLog(false, subsystemName, "setProfileTarget", "Target", position, "Allowed", "TRUE");
    } else {
      log.writeLog(false, subsystemName, "setProfileTarget", "Target", position, "Allowed", "FALSE");
    }
  }

  /**
   * Get the target position that the elevator is trying to move to
   * If the elevator is in manual control mode, returns the actual position
   * If the elevator is not calibrated, returns +10 inches from the lower limit
   * @return target position in inches
   */
  public double getCurrentElevatorTarget() {
    if (isElevatorCalibrated()) {
      // Motion profile control
      if (isElevatorInPositionControl()) {
        return elevatorProfile.getFinalPosition();
      // Manual control mode
      } else {
        return getElevatorPosition();
      }
    } else {
      // TODO determine if this is a proper solution
      return ElevatorConstants.ElevatorPosition.LOWER_LIMIT.value + 10.0;
    }
  }

  /**
   * Zero the elevator encoders only if the elevator is at the lower limit
   */
  public void checkAndZeroElevatorEncoders() {
    if (isElevatorAtLowerLimit()) {
      stopElevatorMotors(); // Ensure PID loop or motion profile will not move to last set position when resetting encoder
      elevatorMotor1.setPosition(ElevatorPosition.LOWER_LIMIT.value);
      elevatorMotor2.setPosition(ElevatorPosition.LOWER_LIMIT.value);
      elevatorCalibrated = true;
  
      log.writeLog(true, subsystemName, "Calibrate and Zero Encoder", "checkAndZeroElevatorEnc");
    }
  }

  /**
   * Get the velocity of the elevator using one of the motors
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return elevator velocity in in/s, (positive = up, negative = down)
   */
  public double getElevatorVelocity(int motor) {
    switch (motor) {
      case 1:
        elevatorMotor1EncoderVelocity.refresh();
        return elevatorMotor1EncoderVelocity.getValueAsDouble() * ElevatorConstants.kElevEncoderInchesPerTick * 10.0;
      case 2:
        elevatorMotor2EncoderVelocity.refresh();
        return elevatorMotor2EncoderVelocity.getValueAsDouble() * ElevatorConstants.kElevEncoderInchesPerTick * 10.0;
      default:
        return -9999;
    }
  }

  /**
   * Get the average velocity of the elevator for the elevator motors
	 * @return double -1.0 to +1.0 (positive = up, negative = down)
	 */
	public double getElevatorVelocity() {
    return (getElevatorVelocity(1) + getElevatorVelocity(2)) / 2;
	}

  /**
   * Get the raw encoder rotations of a specific elevator motor
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return raw encoder reading in pinion rotations, adjusted for direction (positive = up, negative = down)
   */
  public double getElevatorEncoderRotationsRaw(int motor) {
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
   * Get the encoder ticks of a specific elevator motor
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return encoder ticks, based on encoder zero being at lower limit
   */
  public double getElevatorEncoderTicks(int motor) {
    switch (motor) {
      case 1:
        return getElevatorEncoderRotationsRaw(1) * ElevatorConstants.kEncoderCPR;
      case 2:
        return getElevatorEncoderRotationsRaw(2) * ElevatorConstants.kEncoderCPR;
      default:
        return -9999;
    }
  }

  /**
   * Get the vertical position of a specific elevator motor
   * @param motor 1 = elevatorMotor1, 2 = elevatorMotor2
   * @return position in inches, or the lower limit if uncalibrated
   */
  public double getElevatorPosition(int motor) {
    if (!elevatorCalibrated) return ElevatorPosition.LOWER_LIMIT.value;

    switch (motor) {
      case 1:
        return getElevatorEncoderTicks(1) * ElevatorConstants.kElevEncoderInchesPerTick;
      case 2:
        return getElevatorEncoderTicks(2) * ElevatorConstants.kElevEncoderInchesPerTick;
      default:
        return ElevatorPosition.LOWER_LIMIT.value;
    }
  }

   /**
   * Get the vertical position of the elevator using the average position of the motors
   * @return position in inches, or the lower limit if uncalibrated
	 */
	public double getElevatorPosition() {
    return (getElevatorPosition(1) + getElevatorPosition(2)) / 2;
	}

  public double getElevatorTemp(int motor) {
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

  public double getElevatorTemp() {
    return getElevatorTemp(1) / getElevatorTemp(2);
  }

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

  public double getElevatorVoltage() {
    return getElevatorVoltage(1) / getElevatorVoltage(2);
  }

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

  public double getStatorCurrent() {
    return getStatorCurrent(1) / getStatorCurrent(2);
  }

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

  public double getBusVoltage() {
    return getBusVoltage(1) / getBusVoltage(2);
  }

  /**
   * Write information about the subsystem to the FileLog
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
        "Temp 1", getElevatorTemp(1),
        "Voltage 1", getElevatorVoltage(1), "Amps 1", getStatorCurrent(1),
        "EncTicks 1", getElevatorEncoderTicks(1), "EncInches 1", getElevatorPosition(1),
        "PctOut 1", getElevatorPercentOutput(1), "Vel 1", getElevatorVelocity(1),

        "Temp 2", getElevatorTemp(2),
        "Voltage 2", getElevatorVoltage(2), "Amps 2", getStatorCurrent(2),
        "EncTicks 2", getElevatorEncoderTicks(2), "EncInches 2", getElevatorPosition(2),
        "PctOut 2", getElevatorPercentOutput(2), "Vel 2", getElevatorVelocity(2),

        "Elev Calibrated", elevatorCalibrated, "Elev Pos Control", elevatorPositionControl,
        "At Upper Limit", isElevatorAtUpperLimit(), "At Lower Limit", isElevatorAtLowerLimit(),
        "Target", getCurrentElevatorTarget(), "Bus Voltage", getBusVoltage());
  }

  /**
   * Turns file logging on every scheduler cycle (~20ms) or every 10 cycles (~0.2 sec)
   * @param enabled true = every cycle, false = every 10 cycles
   */
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  @Override
  public void periodic() {
    if (log.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Elev Calibrated", isElevatorCalibrated());
      SmartDashboard.putBoolean("Elev Pos Control", isElevatorInPositionControl());

      SmartDashboard.putNumber("Elev Pos", getElevatorPosition());
      SmartDashboard.putNumber("Elev Target", getCurrentElevatorTarget());

      SmartDashboard.putBoolean("Elev Lower Limit", isElevatorAtLowerLimit());
      SmartDashboard.putBoolean("Elev Upper Limit", isElevatorAtUpperLimit());

      SmartDashboard.putNumber("Elev Ticks 1", (getElevatorEncoderTicks(1)));
      SmartDashboard.putNumber("Elev Ticks 2", (getElevatorEncoderTicks(2)));

      SmartDashboard.putNumber("Elev Rotations Raw 1", getElevatorEncoderRotationsRaw(1));
      SmartDashboard.putNumber("Elev Rotations Raw 2", getElevatorEncoderRotationsRaw(2));
    }

    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    // Elevator is not calibrated and is at the lower limit, so we should update encoder calibrations
    if (!isElevatorCalibrated() && isElevatorAtLowerLimit()) {
      checkAndZeroElevatorEncoders();
      log.writeLogEcho(true, subsystemName, "Calibrate Encoders", "Post Elevator Pos", getElevatorPosition());
    }

    // If the elevator is at the lower limit, prevent it from going down any further
    if (isElevatorAtLowerLimit()) {
      // Elevator is calibrated and under position control. If moving down, set the target to the current position.
      if (isElevatorCalibrated() && isElevatorInPositionControl()) {
        if (getCurrentElevatorTarget() < getElevatorPosition()) {
          stopElevatorMotors(); 
          setElevatorProfileTarget(ElevatorPosition.LOWER_LIMIT.value);
        }

      // Elevator is under direct control. If moving down, stop the motors.
      } else if (!isElevatorInPositionControl()) {
        if (getElevatorPercentOutput() < 0) {
          stopElevatorMotors();
        }

      // Elevator is uncalibrated and under position control. Stop the motors.
      } else {
        stopElevatorMotors();
      }
    }

    // Elevator is calibrated and current position is lower than lower limit, so set the target to the lower limit
    if (isElevatorCalibrated() && getElevatorPosition() < ElevatorPosition.LOWER_LIMIT.value) {
      stopElevatorMotors(); 
      setElevatorProfileTarget(ElevatorPosition.LOWER_LIMIT.value);
    }

    // If the elevator is at the upper limit, prevent it from going up any further
    if (isElevatorAtUpperLimit()) {
      // Elevator is calibrated and under position control. If moving up, stop the motors.
      if (isElevatorCalibrated() && isElevatorInPositionControl()) {
        if (getCurrentElevatorTarget() > getElevatorPosition()) {
          stopElevatorMotors(); 
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

    // Elevator is calibrated and current position is higher than upper limit, so stop the motors
    if (isElevatorCalibrated() && getElevatorPosition() > ElevatorPosition.UPPER_LIMIT.value) {
      stopElevatorMotors(); 
    }

    // Update the percent output based on the motion profile
    if (isElevatorInPositionControl()) {
	    setElevatorPercentOutputDirect(elevatorProfile.trackProfilePeriodic());
    }
  }
}
