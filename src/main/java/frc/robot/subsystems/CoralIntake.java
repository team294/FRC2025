// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.utilities.FileLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName; // subsystem name for use in file logging and Shuffleboard

  // Create Kraken variables for coralIntake motor
  private final StatusSignal<Voltage> coralIntakeSupplyVoltage; // Incoming bus voltage to motor contcoralIntake, in volts
  private final StatusSignal<Temperature> coralIntakeTemp; // Motor temperature, in degC
  private final StatusSignal<Double> coralIntakeDutyCycle;	// Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> coralIntakeStatorCurrent; // Motor stator current, in amps (+=fwd, -=rev)
  private final StatusSignal<Angle> coralIntakeEncoderPosition; // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> coralIntakeEncoderVelocity;	
  private final StatusSignal<Voltage> coralIntakeVoltage;

  // Create Kraken for coralIntake motor
  private final TalonFX coralIntakeMotor = new TalonFX(Constants.Ports.CANCoralIntake);
  private final TalonFXConfigurator coralIntakeConfigurator = coralIntakeMotor.getConfigurator();
  private TalonFXConfiguration coralIntakeConfig;
  private VoltageOut coralIntakeVoltageControl = new VoltageOut(0.0); 

  /**
   * Creates the coralIntake subsystem
   * @param subsystemName
   * @param log
   */
  public CoralIntake(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
    coralIntakeSupplyVoltage = coralIntakeMotor.getSupplyVoltage();
    coralIntakeTemp = coralIntakeMotor.getDeviceTemp();
    coralIntakeDutyCycle = coralIntakeMotor.getDutyCycle();
    coralIntakeStatorCurrent = coralIntakeMotor.getStatorCurrent();
    coralIntakeEncoderPosition = coralIntakeMotor.getPosition();
    coralIntakeEncoderVelocity = coralIntakeMotor.getVelocity();
    coralIntakeVoltage = coralIntakeMotor.getMotorVoltage();

    // Configure the coralIntake motor
    coralIntakeConfig = new TalonFXConfiguration(); // Factory default configuration
    coralIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    coralIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralIntakeConfig.Voltage.PeakForwardVoltage = CoralIntakeConstants.compensationVoltage;
    coralIntakeConfig.Voltage.PeakReverseVoltage = -CoralIntakeConstants.compensationVoltage;
    coralIntakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // # seconds from 0 to full power

    // Supply current limit is typically used to prevent breakers from tripping
    coralIntakeConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    coralIntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // (amps) Lower limit for the current
    coralIntakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2; // (sec) Threshold time
    coralIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set coralIntake motor configuration
    coralIntakeConfigurator.apply(coralIntakeConfig);

    stopcoralIntakeMotor();
  }
  

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /** 
   * Sets the percent of the coralIntake motor using voltage compensation
   * @param percent percent 0.0 to 1.0 (+ = outtake coral)
  */
  public void setcoralIntakePercentOutput(double percent) {
    coralIntakeMotor.setControl(coralIntakeVoltageControl.withOutput(percent*CoralIntakeConstants.compensationVoltage));
  }

  public void stopcoralIntakeMotor() {
    setcoralIntakePercentOutput(0);
  }

  /**
   * Get the velocity of the coralIntake motor
   * @return coralIntake motor velocity, in RPM
   */
  public double getcoralIntakeVelocity() {
    coralIntakeEncoderVelocity.refresh();
    return coralIntakeEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Get the current of the coralIntake motor
   * @return coralIntake motor current, in amps
   */
  public double getcoralIntakeAmps() {
    coralIntakeStatorCurrent.refresh();
    return coralIntakeStatorCurrent.getValueAsDouble();
  }


  @Override
  public void periodic() {
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }
  }

  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about coralIntake to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "coralIntake Current (Amps)", getcoralIntakeAmps(),
      "coralIntake Velocity (RPM)", getcoralIntakeVelocity()
    );
  }
}
