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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;

public class EndEffector extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName; // subsystem name for use in file logging and dashboard

  // Create Kraken variables for endEffector motor
  private final StatusSignal<Voltage> endEffectorSupplyVoltage; // Incoming bus voltage to motor, in volts
	private final StatusSignal<Temperature> endEffectorTemp; // Motor temperature, in degC
	private final StatusSignal<Double> endEffectorDutyCycle; // Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> endEffectorStatorCurrent; // Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Angle> endEffectorEncoderPosition; // Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> endEffectorEncoderVelocity;	
	private final StatusSignal<Voltage> endEffectorVoltage;

  // Create Kraken for endEffector motor
  private final TalonFX endEffectorMotor = new TalonFX(Constants.Ports.CANEndEffector);
  private final TalonFXConfigurator endEffectorConfigurator = endEffectorMotor.getConfigurator();
  private TalonFXConfiguration endEffectorConfig;
  private VoltageOut endEffectorVoltageControl = new VoltageOut(0.0);

  // Create entry and exit sensors inside the endEffector
  private final DigitalInput entrySensor = new DigitalInput(Ports.DIOEndEffectorEntrySensor);
  private final DigitalInput exitSensor = new DigitalInput(Ports.DIOEndEffectorExitSensor);

  private double targetPercentOutput = 0;
  
  /**
   * Creates endEffector subsystem
   * @param subsystemName
   * @param log
   */
  public EndEffector(String subsystemName, FileLog log) {
    this.log = log;
    this.subsystemName = subsystemName;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
    endEffectorSupplyVoltage = endEffectorMotor.getSupplyVoltage();
    endEffectorTemp = endEffectorMotor.getDeviceTemp();
    endEffectorDutyCycle = endEffectorMotor.getDutyCycle();
    endEffectorStatorCurrent = endEffectorMotor.getStatorCurrent();
    endEffectorEncoderPosition = endEffectorMotor.getPosition();
    endEffectorEncoderVelocity = endEffectorMotor.getVelocity();
    endEffectorVoltage = endEffectorMotor.getMotorVoltage();

    // Configure the endEffector motor
    endEffectorConfig = new TalonFXConfiguration(); // Factory default configuration
    endEffectorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
	  endEffectorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    endEffectorConfig.Voltage.PeakForwardVoltage = EndEffectorConstants.compensationVoltage;
    endEffectorConfig.Voltage.PeakReverseVoltage = -EndEffectorConstants.compensationVoltage;
    endEffectorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // # seconds from 0 to full power

    // Supply current limit is typically used to prevent breakers from tripping
    endEffectorConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    endEffectorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // (amps) Lower limit for the current
    endEffectorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2; // (sec) Threshold time
    endEffectorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set endEffector motor configuration
    endEffectorConfigurator.apply(endEffectorConfig);

    stopEndEffectorMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /** 
   * Sets the percent of the endEffector motor using voltage compensation
   * @param percent percent 0.0 to 1.0 (+ = outtake coral)
  */
  public void setEndEffectorPercentOutput(double percent) {
    this.targetPercentOutput = percent;
    endEffectorMotor.setControl(endEffectorVoltageControl.withOutput(percent * EndEffectorConstants.compensationVoltage));
  }

  /**
   * Stop the endEffector motor
   */
  public void stopEndEffectorMotor() {
    setEndEffectorPercentOutput(0);
  }

  /**
   * Get the velocity of the endEffector motor
   * @return endEffector motor velocity, in RPM
   */
  public double getEndEffectorVelocity() {
    endEffectorEncoderVelocity.refresh();
    return endEffectorEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Get the current of the endEffector motor
   * @return endEffector motor current, in amps
   */
  public double getEndEffectorAmps() {
    endEffectorStatorCurrent.refresh();
    return endEffectorStatorCurrent.getValueAsDouble();
  }

  /**
   * Get whether a piece is crossing the exit
   * @return true = piece is in exit, false = piece is not in exit
   */
  public boolean isPiecePresentInExit() {
    return !exitSensor.get();
  }

  /**
   * Get whether a piece is crossing the entry
   * @return true = piece is in entry, false = piece is not in entry
   */
  public boolean isPiecePresentInEntry() {
    return entrySensor.get();
  }

  /**
   * Get whether a piece is safely in the end effector (in exit = true + in entry = false)
   * so that the coral is fully out of the way of the elevator path
   * @return true = piece is safe, false = piece is not safe
   */
  public boolean isPieceSafelyIn() {
    return isPiecePresentInExit() && !isPiecePresentInEntry();
  }

  /**
   * Get whether a piece is in the end effector
   * @return true = piece is present, false = piece is not present
   */
  public boolean isPiecePresent() {
    return isPiecePresentInEntry() || isPiecePresentInExit();
  }

  /**
   * Determine if the end effector motor is stalled, based on if there is high current and low rpm
   * and assuming that the motor is running forward (intaking/outtaking)
   * @return true = currently stalled, false = not stalled
   */
  // TODO test and determine if necessary
  public boolean isStalled() {
    boolean highCurrent = getEndEffectorAmps() > EndEffectorConstants.stallThresholdCurrent;
    boolean lowRPM = getEndEffectorVelocity() < EndEffectorConstants.stallThresholdRPM;
    return highCurrent && lowRPM;
  }

  @Override
  public void periodic() {
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);

      SmartDashboard.putBoolean("IsPieceInEntry", isPiecePresentInEntry());
      SmartDashboard.putBoolean("IsPieceInExit", isPiecePresentInExit());
      SmartDashboard.putBoolean("IsPieceSafelyIn", isPieceSafelyIn());
    }

    // If we are trying to run the motor forward and it is stalled, stop the motor
    if (targetPercentOutput > 0 && isStalled()) stopEndEffectorMotor();
  }

  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "EndEffector Current (Amps)", getEndEffectorAmps(),
      "EndEffector Velocity (RPM)", getEndEffectorVelocity()
    );
  }
}