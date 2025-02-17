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
import frc.robot.Constants.HopperConstants;
import frc.robot.utilities.FileLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName; // subsystem name for use in file logging and Shuffleboard

  // Create Kraken variables for hopper motor
  private final StatusSignal<Voltage> hopperSupplyVoltage; // Incoming bus voltage to motor conthopper, in volts
  private final StatusSignal<Temperature> hopperTemp; // Motor temperature, in degC
  private final StatusSignal<Double> hopperDutyCycle;	// Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> hopperStatorCurrent; // Motor stator current, in amps (+=fwd, -=rev)
  private final StatusSignal<Angle> hopperEncoderPosition; // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> hopperEncoderVelocity;	
  private final StatusSignal<Voltage> hopperVoltage;

  // Create Kraken for hopper motor
  private final TalonFX hopperMotor = new TalonFX(Constants.Ports.CANHopper);
  private final TalonFXConfigurator hopperConfigurator = hopperMotor.getConfigurator();
  private TalonFXConfiguration hopperConfig;
  private VoltageOut hopperVoltageControl = new VoltageOut(0.0); 

  /**
   * Creates the hopper subsystem
   * @param subsystemName
   * @param log
   */
  public Hopper(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
    hopperSupplyVoltage = hopperMotor.getSupplyVoltage();
    hopperTemp = hopperMotor.getDeviceTemp();
    hopperDutyCycle = hopperMotor.getDutyCycle();
    hopperStatorCurrent = hopperMotor.getStatorCurrent();
    hopperEncoderPosition = hopperMotor.getPosition();
    hopperEncoderVelocity = hopperMotor.getVelocity();
    hopperVoltage = hopperMotor.getMotorVoltage();

    // Configure the hopper motor
    hopperConfig = new TalonFXConfiguration(); // Factory default configuration
    hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hopperConfig.Voltage.PeakForwardVoltage = HopperConstants.compensationVoltage;
    hopperConfig.Voltage.PeakReverseVoltage = -HopperConstants.compensationVoltage;
    hopperConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // # seconds from 0 to full power

    // Supply current limit is typically used to prevent breakers from tripping
    hopperConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    hopperConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // (amps) Lower limit for the current
    hopperConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2; // (sec) Threshold time
    hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Set hopper motor configuration
    hopperConfigurator.apply(hopperConfig);

    stopHopperMotor();
  }
  

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /** 
   * Sets the percent of the hopper motor using voltage compensation
   * @param percent percent 0.0 to 1.0 (+ = outtake coral)
  */
  public void setHopperPercentOutput(double percent) {
    hopperMotor.setControl(hopperVoltageControl.withOutput(percent*HopperConstants.compensationVoltage));
  }

  public void stopHopperMotor() {
    setHopperPercentOutput(0);
  }

  /**
   * Get the velocity of the hopper motor
   * @return hopper motor velocity, in RPM
   */
  public double getHopperVelocity() {
    hopperEncoderVelocity.refresh();
    return hopperEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Get the current of the hopper motor
   * @return hopper motor current, in amps
   */
  public double getHopperAmps() {
    hopperStatorCurrent.refresh();
    return hopperStatorCurrent.getValueAsDouble();
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
   * Write information about hopper to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "hopper Current (Amps)", getHopperAmps(),
      "hopper Velocity (RPM)", getHopperVelocity()
    );
  }
}
