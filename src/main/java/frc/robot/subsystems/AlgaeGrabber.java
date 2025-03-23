// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.Constants.AlgaeGrabberConstants;
import frc.robot.Constants.Ports;

public class AlgaeGrabber extends SubsystemBase implements Loggable {
  private final DataLogUtil log;
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboard

  // Create variables for the algaeGrabber Minion motor
  private final StatusSignal<Temperature> algaeGrabberTemp;       // Motor temp, in degrees Celsius
	private final StatusSignal<Current> algaeGrabberStatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
	private final StatusSignal<AngularVelocity> algaeGrabberEncoderVelocity;	
	private final StatusSignal<Voltage> algaeGrabberVoltage;

  private final TalonFXS algaeGrabberMotor = new TalonFXS(Ports.CANAlgaeGrabber);
  private final TalonFXSConfigurator algaeGrabberConfigurator = algaeGrabberMotor.getConfigurator();
  private TalonFXSConfiguration algaeGrabberConfig;
  private VoltageOut algaeGrabberVoltageControl = new VoltageOut(0.0);

  // Create bump switches
  private final DigitalInput bumpSwitch = new DigitalInput(Ports.DIOAlgaeGrabberBumpSwitch);

  public AlgaeGrabber(String subsystemName, DataLogUtil log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects
    algaeGrabberTemp = algaeGrabberMotor.getDeviceTemp();
    algaeGrabberStatorCurrent = algaeGrabberMotor.getStatorCurrent();
    algaeGrabberEncoderVelocity = algaeGrabberMotor.getVelocity();
    algaeGrabberVoltage = algaeGrabberMotor.getMotorVoltage();

    // Configure the motor
    algaeGrabberConfig = new TalonFXSConfiguration();
    algaeGrabberConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    algaeGrabberConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Disabled;      // TODO  Should we enable this for the Minion?
    algaeGrabberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeGrabberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeGrabberConfig.Voltage.PeakForwardVoltage = AlgaeGrabberConstants.compensationVoltage;
    algaeGrabberConfig.Voltage.PeakReverseVoltage = -AlgaeGrabberConstants.compensationVoltage;
    algaeGrabberConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3; // Time from 0 to full power, in seconds

    // If the current is above the supply current limit for the threshold time, the current is 
    // limited to the lower limit in order to prevent the breakers from tripping
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLimit = 60.0;       // Upper limit for the current, in amps
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;  // Lower limit for the current, in amps
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;    // Threshold time, in seconds
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    algaeGrabberConfigurator.apply(algaeGrabberConfig);

    stopAlgaeGrabberMotor();
  }

  /** 
   * Sets the percent output of the algaeGrabber motor using voltage compensation. 
   * @param percent output percent, -1.0 to 1.0 (positive = intake, negative = outtake)
   */
  public void setAlgaeGrabberPercentOutput(double percent) {
    algaeGrabberMotor.setControl(algaeGrabberVoltageControl.withOutput(percent * AlgaeGrabberConstants.compensationVoltage));
  }

  /**
   * Stops the algaeGrabber motor.
   */
  public void stopAlgaeGrabberMotor() {
    setAlgaeGrabberPercentOutput(0);
  }

  /** 
   * Gets the velocity of the algaeGrabber motor.
   * @return motor velocity, in RPM
   */
  public double getAlgaeGrabberVelocity() {
    algaeGrabberEncoderVelocity.refresh();
    return algaeGrabberEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Gets the current of the algaeGrabber motor.
   * @return motor current, in amps
   */
  public double getAlgaeGrabberAmps() {
    algaeGrabberStatorCurrent.refresh();
    return algaeGrabberStatorCurrent.getValueAsDouble();
  }

  /**
   * Gets whether an algae is in the algaeGrabber.
   * @return true = algae is present, false = algae is not present or not triggering the bump switch
   */
  public boolean isAlgaePresent() {
    return !bumpSwitch.get();
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
   * Writes information about the algaeGrabber to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update variables",
      "AlgaeGrabber Temp (C)", algaeGrabberTemp.refresh().getValueAsDouble(),
      "AlgaeGrabber Voltage (V)", algaeGrabberVoltage.refresh().getValueAsDouble(),
      "AlgaeGrabber Current (Amps)", getAlgaeGrabberAmps(),
      "AlgaeGrabber Velocity (RPM)", getAlgaeGrabberVelocity());
  }

  @Override
  public void periodic() {
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putBoolean("Algae Present", isAlgaePresent());
      SmartDashboard.putNumber("AlgaeGrabber Velocity", getAlgaeGrabberVelocity());
    }
  }
}
