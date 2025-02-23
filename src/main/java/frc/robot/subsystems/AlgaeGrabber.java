// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
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
import frc.robot.utilities.FileLog;
import frc.robot.Constants.AlgaeGrabberConstants;
import frc.robot.Constants.Ports;

public class AlgaeGrabber extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;  // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;         // Subsystem name for use in file logging and dashboarrd

  // Create variables for the AlgaeGrabber Minion motor
  private final StatusSignal<Voltage> algaeGrabberSupplyVoltage;  // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> algaeGrabberTemp;       // Motor temp, in degrees Celsius
  private final StatusSignal<Double> algaeGrabberDutyCycle;       // Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Current> algaeGrabberStatorCurrent;  // Motor stator current, in amps (positive = forward, negative = reverse)
	private final StatusSignal<Angle> algaeGrabberEncoderPosition;  // Encoder position, in pinion rotations
	private final StatusSignal<AngularVelocity> algaeGrabberEncoderVelocity;	
	private final StatusSignal<Voltage> algaeGrabberVoltage;

  private final TalonFXS algaeGrabberMotor = new TalonFXS(Ports.CANCoralGrabber);
  private final TalonFXSConfigurator algaeGrabberConfigurator = algaeGrabberMotor.getConfigurator();
  private TalonFXSConfiguration algaeGrabberConfig;
  private VoltageOut algaeGrabberVoltageControl;

  // Create bump switches
  private final DigitalInput bumpSwitch1 = new DigitalInput(Ports.DIOAlgaeGrabberBumpSwitch1);
  private final DigitalInput bumpSwitch2 = new DigitalInput(Ports.DIOAlgaeGrabberBumpSwitch2);

  public AlgaeGrabber(String subsystemName, FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    this.subsystemName = subsystemName;

    // Get signal and sensor objects
    algaeGrabberSupplyVoltage = algaeGrabberMotor.getSupplyVoltage();
    algaeGrabberTemp = algaeGrabberMotor.getDeviceTemp();
    algaeGrabberDutyCycle = algaeGrabberMotor.getDutyCycle();
    algaeGrabberStatorCurrent = algaeGrabberMotor.getStatorCurrent();
    algaeGrabberEncoderPosition = algaeGrabberMotor.getPosition();
    algaeGrabberEncoderVelocity = algaeGrabberMotor.getVelocity();
    algaeGrabberVoltage = algaeGrabberMotor.getMotorVoltage();

    // Configure the motor
    algaeGrabberConfig = new TalonFXSConfiguration();
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
   * Sets the percent of the algaeGrabber motor using voltage compensation. 
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
   * @return true = algae is present, false = algae is not present or not triggering either bump switch
   */
  public boolean isAlgaePresent() {
    return bumpSwitch1.get() || bumpSwitch2.get();
  }

  /**
   * Gets whether an algae is triggering a specific bump switch in the algaeGrabber.
   * @param bumpSwitch 1 = bumpSwitch1, 2 = bumpSwitch2
   * @return true = bump switch is triggered, false = bump switch is not triggered
   */
  public boolean isAlgaePresent(int bumpSwitch) {
    if (bumpSwitch == 1) return bumpSwitch1.get();
    else if (bumpSwitch == 2) return bumpSwitch2.get();
    else return false;
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Writes information about the algaeGrabber to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update variables", 
      "AlgaeGrabber Current (Amps)", getAlgaeGrabberAmps(),
      "AlgaeGrabber Velocity (RPM)", getAlgaeGrabberVelocity());
  }

  @Override
  public void periodic() {
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putBoolean("Algae Present", isAlgaePresent());
    }
  }
}
