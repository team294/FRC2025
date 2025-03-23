package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
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
import frc.robot.utilities.Loggable;

public class CoralEffector extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false;    // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;           // Subsystem name for use in file logging and dashboard
    
  private final TalonFXS coralEffectorMotor = new TalonFXS(Ports.CANCoralEffector);

  // Create variables for the coralEffector Minion motor
  private final StatusSignal<Temperature> coralEffectorTemp;                  // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> coralEffectorStatorCurrent;             // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> coralEffectorEncoderPosition;                     // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> coralEffectorEncoderVelocity;
  private final StatusSignal<Voltage> coralEffectorVoltage;
  private final StatusSignal<ControlModeValue> coralEffectorControlMode;

  private final TalonFXSConfigurator coralEffectorConfigurator = coralEffectorMotor.getConfigurator();
  private TalonFXSConfiguration coralEffectorConfig;
  private VoltageOut coralEffectorVoltageControl = new VoltageOut(0.0);
  private PositionVoltage coralEffectorPositionControl = new PositionVoltage(0.0);

  // Create entry and exit banner sensors
  private final DigitalInput entrySensor = new DigitalInput(Ports.DIOCoralEffectorEntrySensor);
  private final DigitalInput exitSensor = new DigitalInput(Ports.DIOCoralEffectorExitSensor);

  private final Wrist wrist;
  private boolean autoHoldMode = false;
  private double targetPosition = 0;

  public CoralEffector(String subsystemName, Wrist wrist, FileLog log) {
    this.subsystemName = subsystemName;
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    this.wrist = wrist;

    // Get signal and sensor objects
    coralEffectorTemp = coralEffectorMotor.getDeviceTemp();
    coralEffectorStatorCurrent = coralEffectorMotor.getStatorCurrent();
    coralEffectorEncoderPosition = coralEffectorMotor.getPosition();
    coralEffectorEncoderVelocity = coralEffectorMotor.getVelocity();
    coralEffectorVoltage = coralEffectorMotor.getMotorVoltage();
    coralEffectorControlMode = coralEffectorMotor.getControlMode();

    // Configure the motor
    coralEffectorConfig = new TalonFXSConfiguration();
    coralEffectorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    coralEffectorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Disabled;      // TODO  Enable this for the Minion after turning on PhoenixPro.  Improves velocity measurement.
    coralEffectorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    coralEffectorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralEffectorConfig.Voltage.PeakForwardVoltage = 2.0;  // Voltage limit needed to cap feedback during PositionVoltage control to prevent oscillation
    coralEffectorConfig.Voltage.PeakReverseVoltage = -2.0;  // Voltage limit needed to cap feedback during PositionVoltage control to prevent oscillation
    coralEffectorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;  // Time from 0 to full power, in seconds
    coralEffectorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3;  // Time from 0 to full power, in seconds

    // If the current is above the supply current limit for the threshold time, the current is limited to the lower limit.
    // This is configured to prevent the breakers from tripping.
    coralEffectorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;        // Upper limit for the current, in amps
    coralEffectorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35.0;   // Lower limit for the current, in amps
    coralEffectorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.2;     // Threshold time, in seconds
    coralEffectorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Configure PID for PositionVoltage control
    coralEffectorConfig.Slot0.kP = CoralEffectorConstants.kP;  // kP = (desired-output-volts) / (error-in-encoder-rotations)
    coralEffectorConfig.Slot0.kI = 0.0;       // kI = (desired-output-volts) / [(error-in-encoder-rotations) * (seconds)]
    coralEffectorConfig.Slot0.kD = 0.0;       // kD = (desired-output-volts) / [(error-in-encoder-rotations) / (seconds)]

    // Configure encoder to user for feedback
    coralEffectorConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    coralEffectorConfig.ExternalFeedback.RotorToSensorRatio = 1.0;
    coralEffectorConfig.ExternalFeedback.SensorToMechanismRatio = 1.0;
    coralEffectorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Set feedback behaviors for position control
    coralEffectorPositionControl.Slot = 0;
    coralEffectorPositionControl.OverrideBrakeDurNeutral = true;
  
    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms for the zero to apply.
    coralEffectorConfigurator.apply(coralEffectorConfig);

    stopCoralEffectorMotor();
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }

  // ********** Motor movement methods

  /**
   * Sets the percent output of the coralEffector motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = intake/outtake, negative = reverse)
   */
  public void setCoralEffectorPercentOutput(double percent) {
    coralEffectorMotor.setControl(coralEffectorVoltageControl.withOutput(percent * CoralEffectorConstants.compensationVoltage));
    autoHoldMode = false;
  }

  /**
   * Stops the coralEffector motor.
   */
  public void stopCoralEffectorMotor() {
    setCoralEffectorPercentOutput(0);
  }

  /**
   * Sets the coralEffector motor to hold a specific position.
   * @param position position to hold, in motor rotations
   * @param autoHold true = automatically adjust position so that both coral sensors detect the coral.  
   *   false = hold exact position specified in first parameter.
   */
  public void setCoralEffectorPosition(double position, boolean autoHold) {
    coralEffectorMotor.setControl(coralEffectorPositionControl.withPosition(position)
          .withFeedForward(-CoralEffectorConstants.kG * Math.cos(Units.degreesToRadians(wrist.getWristAngle()))));  // add an offset angle to true vertical coral, if needed.
    targetPosition = position;
    autoHoldMode = autoHold;
  }

  /**
   * Gets whether the coral motor is in position control or direct percent output control.
   * @return true = position control, false = direct percent output control
   */
  public boolean isMotorPositionControl() {
    return (coralEffectorControlMode.refresh().getValue() == ControlModeValue.PositionVoltage) || 
            (coralEffectorControlMode.refresh().getValue() == ControlModeValue.PositionVoltageFOC);
  }

  // ********** Motor information methods

  /**
   * Gets the angle (position) of the coralEffector motor.
   * @return motor position, in rotations
   */
  public double getCoralEffectorPosition() {
    coralEffectorEncoderPosition.refresh();
    return coralEffectorEncoderPosition.getValueAsDouble();
  }

  /**
   * Gets the velocity of the coralEffector motor.
   * @return motor velocity, in RPM
   */
  public double getCoralEffectorVelocity() {
    coralEffectorEncoderVelocity.refresh();
    return coralEffectorEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Gets the current of the coralEffector motor.
   * @return motor current, in amps
   */
  public double getCoralEffectorAmps() {
    coralEffectorStatorCurrent.refresh();
    return coralEffectorStatorCurrent.getValueAsDouble();
  }

  // ********** Coral sensor methods

  /**
   * Gets whether a coral is in the exit of the coralEffector.
   * @return true = coral is in exit, false = coral is not in exit
   */
  public boolean isCoralPresentInExit() {
    return !exitSensor.get();
  }

  /**
   * Gets whether a coral is in the entry of the coralEffector.
   * @return true = coral is in entry, false = coral is not in entry
   */
  public boolean isCoralPresentInEntry() {
    return !entrySensor.get();
  }

  /**
   * Gets whether a coral is present and is safely in the coralEffector.
   * This occurs when the coral is positioned such that it is in the exit but not in the entry.
   * @return true = coral is safe, false = coral is not safe
   */
  public boolean isCoralSafelyIn() {
    return !isCoralPresentInEntry() && isCoralPresentInExit();
  }

  /**
   * Gets whether a coral is in the coralEffector.
   * @return true = coral is present, false = coral is not present
   */
  public boolean isCoralPresent() {
    return isCoralPresentInEntry() || isCoralPresentInExit();
  }

  // ********** Loggin and periodic methods

  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */
  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Writes information about the coralEffector to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "Temp (C)", coralEffectorTemp.refresh().getValueAsDouble(),
      "Voltage (V)", coralEffectorVoltage.refresh().getValueAsDouble(),
      "Current (Amps)", getCoralEffectorAmps(),
      "Position (rot)", getCoralEffectorPosition(),
      "Velocity (RPM)", getCoralEffectorVelocity(),
      "Position control", isMotorPositionControl(),
      "Target position (rot)", targetPosition,
      "Auto Hold", autoHoldMode
    );
  }

  @Override
  public void periodic() {
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if (log.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Coral in Entry", isCoralPresentInEntry());
      SmartDashboard.putBoolean("Coral in Exit", isCoralPresentInExit());
      SmartDashboard.putBoolean("Coral Safely In", isCoralSafelyIn());
      SmartDashboard.putNumber("Coral Position", getCoralEffectorPosition());
      SmartDashboard.putNumber("Coral Velocity", getCoralEffectorVelocity());
      SmartDashboard.putBoolean("Coral Auto Hold", autoHoldMode);
    }

    // If in coral auto hold mode and is at its target position, 
    // then adjust position target (if needed) to make sure it is held in a safe position
    if (autoHoldMode && Math.abs(targetPosition - getCoralEffectorPosition()) < CoralEffectorConstants.centeringTolerance) {
      // Coral is too far forward, so move it back until it is in a safe position
      if (isCoralPresentInExit() && !isCoralPresentInEntry()) {
        setCoralEffectorPosition(targetPosition - CoralEffectorConstants.centeringStepSize, autoHoldMode);
      }
      // Coral is too far back, so move it forward until it is in a safe position
      if (!isCoralPresentInExit() && isCoralPresentInEntry()) {
        setCoralEffectorPosition(targetPosition + CoralEffectorConstants.centeringStepSize, autoHoldMode);
      }
      // Note: if this is too slow, then set % power slow (0.025%) to center the coral, then set the position
    }
  }
}
