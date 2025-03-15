// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;    // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;           // Subsystem name for use in file logging and dashboard

  private final TalonFX hopperMotor = new TalonFX(Constants.Ports.CANHopper);

  // Create variables for the hopper Kraken motor
  private final StatusSignal<Temperature> hopperTemp;                 // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> hopperStatorCurrent;            // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<AngularVelocity> hopperEncoderVelocity;
  private final StatusSignal<Voltage> hopperVoltage;

  private final TalonFXConfigurator hopperConfigurator = hopperMotor.getConfigurator();
  private TalonFXConfiguration hopperConfig;
  private VoltageOut hopperVoltageControl = new VoltageOut(0.0);

  public Hopper(String subsystemName, FileLog log) {
    this.subsystemName = subsystemName;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
    hopperTemp = hopperMotor.getDeviceTemp();
    hopperStatorCurrent = hopperMotor.getStatorCurrent();
    hopperEncoderVelocity = hopperMotor.getVelocity();
    hopperVoltage = hopperMotor.getMotorVoltage();

    // Configure the motor
    hopperConfig = new TalonFXConfiguration();
    hopperConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hopperConfig.Voltage.PeakForwardVoltage = HopperConstants.compensationVoltage;
    hopperConfig.Voltage.PeakReverseVoltage = -HopperConstants.compensationVoltage;
    hopperConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // If the current is above the supply current limit for the threshold time, the current is limited to the lower limit.
    // This is configured to prevent the breakers from tripping.
    hopperConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    hopperConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    hopperConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms for the zero to apply.
    hopperConfigurator.apply(hopperConfig);

    stopHopperMotor();
  }

  /**
   * Sets the percent output of the hopper motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = intake, negative = reverse)
   */
  public void setHopperPercentOutput(double percent) {
    hopperMotor.setControl(hopperVoltageControl.withOutput(percent * HopperConstants.compensationVoltage));
  }

  /**
   * Stops the hopper motor.
   */
  public void stopHopperMotor() {
    setHopperPercentOutput(0);
  }

  /**
   * Gets the velocity of the hopper motor.
   * @return motor velocity, in RPM
   */
  public double getHopperVelocity() {
    hopperEncoderVelocity.refresh();
    return hopperEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Gets the current of the hopper motor.
   * @return motor current, in amps
   */
  public double getHopperAmps() {
    hopperStatorCurrent.refresh();
    return hopperStatorCurrent.getValueAsDouble();
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
   * Write information about the hopper to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "Hopper Temp (C)", hopperTemp.refresh().getValueAsDouble(),
      "Hopper Voltage (V)", hopperVoltage.refresh().getValueAsDouble(),
      "Hopper Current (Amps)", getHopperAmps(),
      "Hopper Velocity (RPM)", getHopperVelocity()
    );
  }

  @Override
  public void periodic() {
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      // updateLog(false);
      SmartDashboard.putNumber("Hopper Velocity", getHopperVelocity());
    }
  }
}
