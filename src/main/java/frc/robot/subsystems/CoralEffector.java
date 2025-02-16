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
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;

public class CoralEffector extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;    // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;           // Subsystem name for use in file logging and dashboard
    
  private final TalonFXS coralEffectorMotor = new TalonFXS(Ports.CANCoralEffector);

  // Create variables for the coralEffectorMotor
  private final StatusSignal<Voltage> coralEffectorSupplyVoltage;             // Incoming bus voltage to motor, in volts
  private final StatusSignal<Temperature> coralEffectorTemp;                  // Motor temperature, in degrees Celsius
  private final StatusSignal<Double> coralEffectorDutyCycle;                  // Motor duty cycle percent power, -1 to 1
  private final StatusSignal<Current> coralEffectorStatorCurrent;             // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> coralEffectorEncoderPosition;             // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> coralEffectorEncoderVelocity;
  private final StatusSignal<Voltage> coralEffectorVoltage;

  private final TalonFXSConfigurator coralEffectorConfigurator = coralEffectorMotor.getConfigurator();
  private TalonFXSConfiguration coralEffectorConfig;
  private VoltageOut coralEffectorVoltageControl = new VoltageOut(0.0);

  // Create entry and exit banner sensors
  private final DigitalInput entrySensor = new DigitalInput(Ports.DIOCoralEffectorEntrySensor);
  private final DigitalInput exitSensor = new DigitalInput(Ports.DIOCoralEffectorExitSensor);

  private double targetPercentOutput = 0;

  public CoralEffector(String subsystemName, FileLog log) {
    this.subsystemName = subsystemName;
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
    coralEffectorSupplyVoltage = coralEffectorMotor.getSupplyVoltage();
    coralEffectorTemp = coralEffectorMotor.getDeviceTemp();
    coralEffectorDutyCycle = coralEffectorMotor.getDutyCycle();
    coralEffectorStatorCurrent = coralEffectorMotor.getStatorCurrent();
    coralEffectorEncoderPosition = coralEffectorMotor.getPosition();
    coralEffectorEncoderVelocity = coralEffectorMotor.getVelocity();
    coralEffectorVoltage = coralEffectorMotor.getMotorVoltage();

    // Configure the motor
    coralEffectorConfig = new TalonFXSConfiguration();
    coralEffectorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    coralEffectorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralEffectorConfig.Voltage.PeakForwardVoltage = CoralEffectorConstants.compensationVoltage;
    coralEffectorConfig.Voltage.PeakReverseVoltage = -CoralEffectorConstants.compensationVoltage;
    coralEffectorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;  // Time from 0 to full power, in seconds

    // If the current is above the supply current limit for the threshold time, the current is limited to the lower limit.
    // This is configured to prevent the breakers from tripping.
    coralEffectorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // Upper limit for the current, in amps
    coralEffectorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;   // Lower limit for the current, in amps
    coralEffectorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;     // Threshold time, in seconds
    coralEffectorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply the configurations to the motor
    coralEffectorConfigurator.apply(coralEffectorConfig);

    stopCoralEffectorMotor();
  }

  /**
   * Sets the percent of the coralEffector motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = intake/outtake, negative = reverse)
   */
  public void setCoralEffectorPercentOutput(double percent) {
    this.targetPercentOutput = percent;
    coralEffectorMotor.setControl(coralEffectorVoltageControl.withOutput(percent * CoralEffectorConstants.compensationVoltage));
  }

  /**
   * Stop the coralEffector motor.
   */
  public void stopCoralEffectorMotor() {
    setCoralEffectorPercentOutput(0);
  }

  /**
   * Get the velocity of the coralEffector motor.
   * @return motor velocity, in RPM
   */
  public double getCoralEffectorVelocity() {
    coralEffectorEncoderVelocity.refresh();
    return coralEffectorEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Get the current of the coralEffector motor.
   * @return motor current, in amps
   */
  public double getCoralEffectorAmps() {
    coralEffectorStatorCurrent.refresh();
    return coralEffectorStatorCurrent.getValueAsDouble();
  }

  /**
   * Get whether a coral is in the exit of the coralEffector.
   * @return true = coral is in exit, false = coral is not in exit
   */
  public boolean isCoralPresentInExit() {
    return !exitSensor.get();
  }

  /**
   * Get whether a coral is in the entry of the coralEffector.
   * @return true = coral is in entry, false = coral is not in entry
   */
  public boolean isCoralPresentInEntry() {
    return entrySensor.get();
  }

  /**
   * Get whether a coral is safely in the coralEffector.
   * This occurs when the coral is positioned such that it is in the exit but not in the entry.
   * If there is not a coral present, this will return false. TODO is this behavior we want?
   * @return true = coral is safe, false = coral is not safe
   */
  public boolean isCoralSafelyIn() {
    return !isCoralPresentInEntry() && isCoralPresentInExit();
  }

  /**
   * Get whether a coral is in the coralEffector.
   * @return true = coral is present, false = coral is not present
   */
  public boolean isPiecePresent() {
    return isCoralPresentInEntry() || isCoralPresentInExit();
  }

  /**
   * Get whether the coralEffector motor is stalled, which occurs when there is high current and low velocity.
   * This detection is only valid when the motor is running forward (intaking/outtaking).
   * @return true = motor is stalled, false = motor is not stalled
   */
  public boolean isStalled() {
    boolean highCurrent = getCoralEffectorAmps() > CoralEffectorConstants.stallThresholdCurrent;
    boolean lowRPM = getCoralEffectorVelocity() < CoralEffectorConstants.stallThresholdRPM;
    return (targetPercentOutput > 0) && highCurrent && lowRPM;
  }

  /**
   * Get the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Write to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "CoralEffector Current (Amps)", getCoralEffectorAmps(),
      "CoralEffector Velocity (RPM)", getCoralEffectorVelocity()
    );
  }

  @Override
  public void periodic() {
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putBoolean("Coral In Entry", isCoralPresentInEntry());
      SmartDashboard.putBoolean("Coral In Exit", isCoralPresentInExit());
      SmartDashboard.putBoolean("Coral Safely In", isCoralSafelyIn());
    }

    // If we are trying to run the motor forward and it is stalled, stop the motor
    if (targetPercentOutput > 0 && isStalled()) stopCoralEffectorMotor();
  }
}
