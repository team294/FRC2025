// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.Constants;


public class AlgaeGrabber extends SubsystemBase {
  
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;
  private String subsystemName;

  //Create minion variables for AlgaeGrabber motor
    private final StatusSignal<Voltage> algaeGrabberSupplyVoltage; // Incoming bus voltage to motor, in volts
    private final StatusSignal<Temperature> algaeGrabberTemp; // Motor temp, in degC
    private final StatusSignal<Double> algaeGrabberDutyCycle; // Motor duty cycle percent power, -1 to 1
	  private final StatusSignal<Current> algaeGrabberStatorCurrent; // Motor stator current, in amps (+=fwd, -=rev)
	  private final StatusSignal<Angle> algaeGrabberEncoderPosition; // Encoder position, in pinion rotations
	  private final StatusSignal<AngularVelocity> algaeGrabberEncoderVelocity;	
	  private final StatusSignal<Voltage> algaeGrabberVoltage;

  // Create minion for AlgaeGrabber motor
    private final TalonFX algaeGrabberMotor = new TalonFX(Constants.Ports.CANMinion1);
    private final TalonFXConfigurator algaeGrabberConfigurator = algaeGrabberMotor.getConfigurator();
    private TalonFXConfiguration algaeGrabberConfig;
    private VoltageOut algaeGrabberVoltageControl;

  // Create sensors for AlgaeGrabber
    private final DigitalInput sensor1 = new DigitalInput(Constants.Ports.DIOAlgaeGrabberSensor1);
    private final DigitalInput sensor2 = new DigitalInput(Constants.Ports.DIOAlgaeGraberSensor2);

    private double targetPercentOutput = 0;

  /** Creates AlgaeGrabber Subsystem
   * @param subsystemName
   * @param log
   */
  public AlgaeGrabber(String subsystemName, FileLog log) {
    this.subsystemName = subsystemName;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
      algaeGrabberSupplyVoltage = algaeGrabberMotor.getSupplyVoltage();
      algaeGrabberTemp = algaeGrabberMotor.getDeviceTemp();
      algaeGrabberDutyCycle = algaeGrabberMotor.getDutyCycle();
      algaeGrabberStatorCurrent = algaeGrabberMotor.getStatorCurrent();
      algaeGrabberEncoderPosition = algaeGrabberMotor.getPosition();
      algaeGrabberEncoderVelocity = algaeGrabberMotor.getVelocity();
      algaeGrabberVoltage = algaeGrabberMotor.getMotorVoltage();

    // Supply current limit is typically used to prevent breakers from tripping, these values gotten from krakren motors, maybe different for minions
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLimit = 60.0; // (amps) If current is above this value for longer than threshold time, then limit current to the lower limit
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0; // (amps) Lower limit for the current
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2; // (sec) Threshold time
    algaeGrabberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    algaeGrabberConfigurator.apply(algaeGrabberConfig);

  }

  public String getName() {
    return subsystemName;
  }

  /** Sets the percent of the algaeGrabber motor using voltage compensation 
   * @param percent percent 0.0 to 1.0 (+= intake algae)
   */
  public void setAlgaeGrabberPercentOutput(double percent) {
    this.targetPercentOutput = percent;
    algaeGrabberMotor.setControl(algaeGrabberVoltageControl.withOutput(percent * Constants.AlgaeGrabberConstants.compensationVoltage));
  }

  /**
   * Stops the algaeGrabber motor
   */
  public void stopAlgaeGrabberMotors() {
    setAlgaeGrabberPercentOutput(0);
  }

  /** Gets the velocity of the algaeGrabber motor
   * @return algaeGrabber motor veloity, in RPM
   */
  public double getAlgaeGrabberVelocity() {
    algaeGrabberEncoderVelocity.refresh();
    return algaeGrabberEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /** Gets the current amps of the algaeGrabber motor
   * @return algaeGrabber motor current, in amps
   */
  public double getAlgaeGrabberAmps() {
    algaeGrabberStatorCurrent.refresh();
    return algaeGrabberStatorCurrent.getValueAsDouble();
  }

  /** Gets if one of the bump switches detects an algae
   * @return true = algae is detected, false = neither switch tripped by an algae
   */
  public boolean isAlgaePresent() {
    return sensor1.get() || sensor2.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IsAlgaePresent", isAlgaePresent());
  }

  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update variables", "AlgaeGrabber Current (Amps)", getAlgaeGrabberAmps(),
    "AlgaeGrabber Velocity (RPM)", getAlgaeGrabberVelocity());
  }
}
