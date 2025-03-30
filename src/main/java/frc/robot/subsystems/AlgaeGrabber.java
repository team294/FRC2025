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

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;
import frc.robot.Constants.AlgaeGrabberConstants;
import frc.robot.Constants.Ports;

public class AlgaeGrabber extends SubsystemBase implements Loggable {
  
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboard

  // Create variables for the algaeGrabber Minion motor
  private final StatusSignal<Temperature> algaeGrabberTemp;       // Motor temp, in degrees Celsius
	private final StatusSignal<Current> algaeGrabberStatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
	private final StatusSignal<AngularVelocity> algaeGrabberEncoderVelocity;	
	private final StatusSignal<Voltage> algaeGrabberVoltage;

  private final TalonFX algaeGrabberMotor = new TalonFX(Ports.CANAlgaeGrabber);
  private final TalonFXConfigurator algaeGrabberConfigurator = algaeGrabberMotor.getConfigurator();
  private TalonFXConfiguration algaeGrabberConfig;
  private VoltageOut algaeGrabberVoltageControl = new VoltageOut(0.0);

  // Create bump switches
  private final DigitalInput bumpSwitch = new DigitalInput(Ports.DIOAlgaeGrabberBumpSwitch);

  // Create Data Log Entries
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTemp = new DoubleLogEntry(log, "/AlgaeGrabber/Temperature");
  private final DoubleLogEntry dLogVoltage = new DoubleLogEntry(log, "/AlgaeGrabber/Voltage");
  private final DoubleLogEntry dLogCurrent = new DoubleLogEntry(log, "/AlgaeGrabber/Current");
  private final DoubleLogEntry dLogVelocity = new DoubleLogEntry(log, "/AlgaeGrabber/Velocity");

  private boolean netScoreMode = true;

  public AlgaeGrabber(String subsystemName) {
    
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects
    algaeGrabberTemp = algaeGrabberMotor.getDeviceTemp();
    algaeGrabberStatorCurrent = algaeGrabberMotor.getStatorCurrent();
    algaeGrabberEncoderVelocity = algaeGrabberMotor.getVelocity();
    algaeGrabberVoltage = algaeGrabberMotor.getMotorVoltage();

    // Configure the motor
    algaeGrabberConfig = new TalonFXConfiguration();
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
   * Sets whether we are scoring in the net or processor.
   * @param net true = scoring in net, false = scoring in processor
   */
  public void setNetScoreMode(boolean net) {
    netScoreMode = net;
  }

  /**
   * Gets whether we are scoring in the net or processor.
   * @return true = scoring in net, false = scoring in processor
   */
  public boolean getNetScoreMode() {
    return netScoreMode;
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
    if(logWhenDisabled || !DriverStation.isDisabled()){
      long timeNow = RobotController.getFPGATime();

      dLogTemp.append(algaeGrabberTemp.refresh().getValueAsDouble(), timeNow);
      dLogVoltage.append(algaeGrabberVoltage.refresh().getValueAsDouble(), timeNow);
      dLogCurrent.append(getAlgaeGrabberAmps(), timeNow);
      dLogVelocity.append(getAlgaeGrabberVelocity(), timeNow);
    }
  }

  @Override
  public void periodic() {
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putBoolean("Algae Present", isAlgaePresent());
      SmartDashboard.putNumber("AlgaeGrabber Velocity", getAlgaeGrabberVelocity());
    }
  }
}
