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

import edu.wpi.first.math.util.Units;
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
  private final StatusSignal<AngularVelocity> coralEffectorEncoderVelocity;
  private final StatusSignal<Voltage> coralEffectorVoltage;

  private final TalonFXSConfigurator coralEffectorConfigurator = coralEffectorMotor.getConfigurator();
  private TalonFXSConfiguration coralEffectorConfig;
  private VoltageOut coralEffectorVoltageControl = new VoltageOut(0.0);

  // Create entry and exit banner sensors
  private final DigitalInput entrySensor = new DigitalInput(Ports.DIOCoralEffectorEntrySensor);
  private final DigitalInput exitSensor = new DigitalInput(Ports.DIOCoralEffectorExitSensor);

  private final Wrist wrist;
  private boolean coralHoldMode = false;

  public CoralEffector(String subsystemName, Wrist wrist, FileLog log) {
    this.subsystemName = subsystemName;
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    this.wrist = wrist;

    // Get signal and sensor objects
    coralEffectorTemp = coralEffectorMotor.getDeviceTemp();
    coralEffectorStatorCurrent = coralEffectorMotor.getStatorCurrent();
    coralEffectorEncoderVelocity = coralEffectorMotor.getVelocity();
    coralEffectorVoltage = coralEffectorMotor.getMotorVoltage();

    // Configure the motor
    coralEffectorConfig = new TalonFXSConfiguration();
    coralEffectorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    coralEffectorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Disabled;      // TODO  Should we enable this for the Minion?
    coralEffectorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms for the zero to apply.
    coralEffectorConfigurator.apply(coralEffectorConfig);

    stopCoralEffectorMotor();
  }

  /**
   * Sets the percent output of the coralEffector motor using voltage compensation.
   * @param percent output percent, -1.0 to 1.0 (positive = intake/outtake, negative = reverse)
   */
  public void setCoralEffectorPercentOutput(double percent) {
    coralEffectorMotor.setControl(coralEffectorVoltageControl.withOutput(percent * CoralEffectorConstants.compensationVoltage));
  }

  /**
   * Stops the coralEffector motor.
   */
  public void stopCoralEffectorMotor() {
    setCoralEffectorPercentOutput(0);
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

  /**
   * Turns on or off coral hold mode whether or not we want to hold coral in place.
   * @param newState true = turn on coralHoldMode, false = turn off coralHoldMode
   */
  public void setCoralHoldMode(boolean newState) {
    coralHoldMode = newState;
    // If turning off coral hold mode, turn off effector motors as well
    if (!newState) stopCoralEffectorMotor();
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
   * Writes information about the coralEffector to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, subsystemName, "Update Variables",
      "CoralEffector Temp (C)", coralEffectorTemp.refresh().getValueAsDouble(),
      "CoralEffector Voltage (V)", coralEffectorVoltage.refresh().getValueAsDouble(),
      "CoralEffector Current (Amps)", getCoralEffectorAmps(),
      "CoralEffector Velocity (RPM)", getCoralEffectorVelocity()
    );
  }

  @Override
  public void periodic() {
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putBoolean("Coral in Entry", isCoralPresentInEntry());
      SmartDashboard.putBoolean("Coral in Exit", isCoralPresentInExit());
      SmartDashboard.putBoolean("Coral Safely In", isCoralSafelyIn());
      SmartDashboard.putNumber("Coral Velocity", getCoralEffectorVelocity());
    }

    // If we need to keep hold of the coral, make sure it is held in a safe position
    if (coralHoldMode) {
      // Coral is too far forward, so move it back until it is in a safe position
      if (isCoralPresentInExit() && !isCoralPresentInEntry()) {
        setCoralEffectorPercentOutput(-CoralEffectorConstants.centeringPercent);
      }
      // Coral is too far back, so move it forward until it is in a safe position
      if (!isCoralPresentInExit() && isCoralPresentInEntry()) {
        setCoralEffectorPercentOutput(CoralEffectorConstants.centeringPercent);
      }
      // Coral is in a safe position, so apply power to keep it in place
      if (isCoralPresentInEntry() && isCoralPresentInExit()) {
        setCoralEffectorPercentOutput(CoralEffectorConstants.holdingPercent * Math.cos(Units.degreesToRadians(wrist.getWristAngle()))); 
      }
    }
  }
}
