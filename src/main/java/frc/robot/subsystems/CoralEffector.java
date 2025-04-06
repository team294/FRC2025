package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.Loggable;

public class CoralEffector extends SubsystemBase implements Loggable {
  
  private final int logRotationKey;
  private boolean fastLogging = false;    // true = enabled to run every cycle, false = follow normal logging cycles
  private String subsystemName;           // Subsystem name for use in file logging and dashboard
    
  private final TalonFX coralEffectorMotor = new TalonFX(Ports.CANCoralEffector);

  // Create variables for the coralEffector Minion motor
  private final StatusSignal<Temperature> coralEffectorTemp;                  // Motor temperature, in degrees Celsius
  private final StatusSignal<Current> coralEffectorStatorCurrent;             // Motor stator current, in amps (positive = forward, negative = reverse)
  private final StatusSignal<Angle> coralEffectorEncoderPosition;                     // Encoder position, in pinion rotations
  private final StatusSignal<AngularVelocity> coralEffectorEncoderVelocity;
  private final StatusSignal<Voltage> coralEffectorVoltage;
  private final StatusSignal<ControlModeValue> coralEffectorControlMode;

  private final TalonFXConfigurator coralEffectorConfigurator = coralEffectorMotor.getConfigurator();
  private TalonFXConfiguration coralEffectorConfig;
  private VoltageOut coralEffectorVoltageControl = new VoltageOut(0.0);
  private PositionVoltage coralEffectorPositionControl = new PositionVoltage(0.0);

  // Create exit banner sensor
  private final DigitalInput exitSensor = new DigitalInput(Ports.DIOCoralEffectorExitSensor);

  private final Wrist wrist;
  private boolean autoHoldMode = false;
  private double targetPosition = 0;

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogTemp = new DoubleLogEntry(log, "/CoralEffector/Temperature");
  private final DoubleLogEntry dLogVoltage = new DoubleLogEntry(log, "/CoralEffector/Voltage");
  private final DoubleLogEntry dLogCurrent = new DoubleLogEntry(log, "/CoralEffector/Current");
  private final DoubleLogEntry dLogVelocity = new DoubleLogEntry(log, "/CoralEffector/Velocity");
  private final DoubleLogEntry dLogPosition = new DoubleLogEntry(log, "/CoralEffector/Position");
  private final DoubleLogEntry dLogTargetPos = new DoubleLogEntry(log, "/CoralEffector/TargetPosition");
  private final BooleanLogEntry dLogPositionControl = new BooleanLogEntry(log, "/CoralEffector/PositionControl");
  private final BooleanLogEntry dLogAutoHold = new BooleanLogEntry(log, "/CoralEffector/AutoHold");
  private final BooleanLogEntry dLogCoralIn = new BooleanLogEntry(log, "/CoralEffector/CoralIn");

  public CoralEffector(String subsystemName, Wrist wrist) {
    this.subsystemName = subsystemName;
    
    logRotationKey = DataLogUtil.allocateLogRotation();
    this.wrist = wrist;

    // Get signal and sensor objects
    coralEffectorTemp = coralEffectorMotor.getDeviceTemp();
    coralEffectorStatorCurrent = coralEffectorMotor.getStatorCurrent();
    coralEffectorEncoderPosition = coralEffectorMotor.getPosition();
    coralEffectorEncoderVelocity = coralEffectorMotor.getVelocity();
    coralEffectorVoltage = coralEffectorMotor.getMotorVoltage();
    coralEffectorControlMode = coralEffectorMotor.getControlMode();

    // Configure the motor
    coralEffectorConfig = new TalonFXConfiguration();
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
    coralEffectorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    coralEffectorConfig.Feedback.RotorToSensorRatio = 1.0;
    coralEffectorConfig.Feedback.SensorToMechanismRatio = 1.0;
    coralEffectorConfig.ClosedLoopGeneral.ContinuousWrap = false;

    // Set feedback behaviors for position control
    coralEffectorPositionControl.Slot = 0;
    coralEffectorPositionControl.OverrideBrakeDurNeutral = true;
  
    // Apply the configurations to the motor.
    // This is a blocking call and will wait up to 200ms for the zero to apply.
    coralEffectorConfigurator.apply(coralEffectorConfig);

    stopCoralEffectorMotor();

    // Prime the DataLog to reduce delay when first enabling the robot
    updateLog(true);
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
   * @param position position to hold, in motor rotations (+ more towards front of robot, - = more towards hopper)
   * @param autoHold true = automatically adjust position so that exit coral sensor detects the coral.  
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

  /**
   * Gets if the endEffector is in hold mode
   * @return the hold mode of the endeffector: True = hold mode, False = not in hold mode
   */
  public boolean getHoldMode(){
    return autoHoldMode;
  }

  // ********** Coral sensor methods

  /**
   * Gets whether a coral is in the coralEffector.
   * @return true = coral is present, false = coral is not present
   */
  public boolean isCoralPresent() {
    return !exitSensor.get();
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
    if (logWhenDisabled || !DriverStation.isDisabled()) {
      long timeNow = RobotController.getFPGATime();

      dLogTemp.append(coralEffectorTemp.refresh().getValueAsDouble(), timeNow);
      dLogVoltage.append(coralEffectorVoltage.refresh().getValueAsDouble(), timeNow);
      dLogCurrent.append(getCoralEffectorAmps(), timeNow);
      dLogVelocity.append(getCoralEffectorVelocity(), timeNow);
      dLogPosition.append(getCoralEffectorPosition(), timeNow);
      dLogTargetPos.append(targetPosition, timeNow);
      dLogPositionControl.append(isMotorPositionControl(), timeNow);
      dLogAutoHold.append(autoHoldMode, timeNow);
      dLogCoralIn.append(isCoralPresent(), timeNow);
    }
  }

  @Override
  public void periodic() {
    if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if (DataLogUtil.isMyLogRotation(logRotationKey)) {
      SmartDashboard.putBoolean("Coral in", isCoralPresent());
      SmartDashboard.putNumber("Coral Position", getCoralEffectorPosition());
      SmartDashboard.putNumber("Coral Velocity", getCoralEffectorVelocity());
      SmartDashboard.putBoolean("Coral Auto Hold", autoHoldMode);
    }

    // If in coral auto hold mode and is at its target position, 
    // then adjust position target (if needed) to make sure it is held in a safe position
    if (autoHoldMode && Math.abs(targetPosition - getCoralEffectorPosition()) < CoralEffectorConstants.centeringTolerance) {
      // Coral is too far back, so move it forward until it is in a safe position
      if (!isCoralPresent()) {
        setCoralEffectorPosition(targetPosition + CoralEffectorConstants.centeringStepSize, autoHoldMode);
      }
      // Note: if this is too slow, then set % power slow (0.025%) to center the coral, then set the position
    }
  }
}
