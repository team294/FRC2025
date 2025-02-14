// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Define Key robot utilities (DO THIS FIRST)
  private final FileLog log = new FileLog("A1");
  private final AllianceSelection allianceSelection = new AllianceSelection(log);
  private final Field field = new Field(allianceSelection, log);
  private final Timer matchTimer = new Timer();

  // Define robot subsystems
  private final LED led = new LED(Constants.Ports.CANdle, "LED", matchTimer, log);
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection, led, log);
  private final EndEffector endEffector = new EndEffector("EndEffector", log);
  private final Elevator elevator = new Elevator(endEffector, "Elevator", log);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, allianceSelection, log);

  // Define controllers
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);
  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }

    // Set the Choreo trajectory flipper to the 2025 field orientation.
    // So, when the alliance is Red, Choreo will automatically flips Blue trajectories to Red.
    // All trajectories should be defined based on the Blue alliance perspective.
    ChoreoAllianceFlipUtil.setYear(2025);

    configureButtonBindings();
    configureShuffleboard();

    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvanced(leftJoystick, rightJoystick, allianceSelection, driveTrain, log));

    // driveTrain.setDefaultCommand(new DriveWithJoysticks(leftJoystick, rightJoystick, allianceSelection, driveTrain, log));
    // driveTrain.setDefaultCommand(new DriveWithController(driveTrain, xboxController, allianceSelection, log));

    // Set initial robot position on field. This takes place a while after the drivetrain is created, so after any CANbus delays.
    // Set initial location to (0,0) and facing away from driver (regardless of alliance color).
    driveTrain.resetPose(new Pose2d(0.0, 0.0, 
      allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero) );
  }

  private void configureShuffleboard() {
    // Display sticky faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));
  
    // Drive base commands
    SmartDashboard.putData("Drive FOC On", new DriveSetFOC(true, driveTrain, log));
    SmartDashboard.putData("Drive FOC Off", new DriveSetFOC(false, driveTrain, log));
    SmartDashboard.putData("Drive Toggle Coast", new DriveToggleCoastMode(driveTrain, log));
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain, log));
    SmartDashboard.putData("Drive To Pose", new DriveToPose(driveTrain, log));
    SmartDashboard.putData("Drive 6m +X", new DriveToPose(
      () -> driveTrain.getPose().plus(new Transform2d(6.0, 0.0, new Rotation2d(0.0))), 
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      false, false, driveTrain, log) );

    SmartDashboard.putData("Drive Calibration", new DriveCalibration(0.0, 0.5, 5.0, 0.1, driveTrain, log));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain, log));
    SmartDashboard.putData("Drive Percent Speed", new DrivePercentSpeed(driveTrain, log));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain, log));

    // Elevator commands
    SmartDashboard.putData("Move Elevator Up", new ElevatorSetPercent(.05, elevator, endEffector, log));
    SmartDashboard.putData("Move Elevator Down", new ElevatorSetPercent(-.05, elevator, endEffector, log));
    SmartDashboard.putData("Stop Elevator Motors", new StopElevatorMotors(elevator, log));
    SmartDashboard.putData("Move Elevator To 20 Inches", new ElevatorSetPosition(20.0, elevator, endEffector, log));
    SmartDashboard.putData("Move Elevator To L2", new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_REEF_L2.value, elevator, endEffector, log));
    SmartDashboard.putData("Move Elevator To L3", new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_REEF_L3.value, elevator, endEffector, log));
    SmartDashboard.putData("Move Elevator To L4", new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_REEF_L4.value, elevator, endEffector, log));
    SmartDashboard.putData("Elevator Calibration", new ElevatorCalibration(0.1, elevator, log));

    // EndEffector commands
    SmartDashboard.putData("EndEffector Run Forward", new EndEffectorSetPercent(EndEffectorConstants.endEffectorIntakePercent, endEffector, log));
    SmartDashboard.putData("EndEffector Run Reverse", new EndEffectorSetPercent(-1 * EndEffectorConstants.endEffectorIntakePercent, endEffector, log));
    SmartDashboard.putData("EndEffector Stop", new StopEndEffectorMotor(endEffector, log));
    SmartDashboard.putData("EndEffector Intake", new EndEffectorCoralIntake(endEffector, log));
    SmartDashboard.putData("EndEffector Outtake", new EndEffectorCoralOuttake(endEffector, log));

    // Autos

    // Copanel buttons

    // Vision
    SmartDashboard.putData("Vision Enable Odometry Updates", new VisionOdometryStateSet(true, driveTrain, log));
    SmartDashboard.putData("Vision Disable Odometry Updates", new VisionOdometryStateSet(false, driveTrain, log));
  }
 
   private void configureButtonBindings() {
    configureXboxButtons(); // configure Xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
    configureTriggers();
  }

  /**
   * Configures Xbox buttons and controls
   */
  private void configureXboxButtons(){
    // Triggers for all Xbox buttons
    Trigger xbLT = xboxController.leftTrigger();
    Trigger xbRT = xboxController.rightTrigger();
    Trigger xbA = xboxController.a();
    Trigger xbB = xboxController.b();
    Trigger xbY = xboxController.y();
    Trigger xbX = xboxController.x();
    Trigger xbLB = xboxController.leftBumper();
    Trigger xbRB = xboxController.rightBumper();
    Trigger xbBack = xboxController.back();
    Trigger xbStart = xboxController.start();
    Trigger xbPOVUp = xboxController.povUp();
    Trigger xbPOVRight = xboxController.povRight();
    Trigger xbPOVLeft = xboxController.povLeft();
    Trigger xbPOVDown = xboxController.povDown();
    Trigger xbRJoystickTrigger = xboxController.rightStick();
    Trigger xbLJoystickTrigger = xboxController.leftStick();

    // *variable name of button*.onTrue(*command(s)*);
    // ex: xbA.onTrue(new command(param1, param2));

    // Move elevator to Home/Intake (X button), L2 (A button), L3 (B button), and L4 (Y button)
    xbX.onTrue(new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_HP.value, elevator, endEffector, log));
    xbA.onTrue(new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_REEF_L2.value, elevator, endEffector, log));
    xbB.onTrue(new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_REEF_L3.value, elevator, endEffector, log));
    xbY.onTrue(new ElevatorSetPosition(ElevatorConstants.ElevatorPosition.CORAL_REEF_L4.value, elevator, endEffector, log));

    // Manually control elevator with right joystick
    xbRJoystickTrigger.whileTrue(new ElevatorManualControl(xboxController, elevator, log, true));

    // Stop all motors
    xbStart.onTrue(parallel(
      new StopElevatorMotors(elevator, log),
      new StopEndEffectorMotor(endEffector, log)
    ));

    // Intake coral with the end effector when RB is pressed
    xbRB.onTrue(new EndEffectorCoralIntake(endEffector, log));

    // Outtake coral with the end effector when RT is pressed
    xbRT.onTrue(new EndEffectorCoralOuttake(endEffector, log));
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // ex: left[1].onTrue(new command);

    // Reset pose
    left[1].onTrue(either(
      new DriveResetPose(180, false, driveTrain, log), 
      new DriveResetPose(0, false, driveTrain, log), 
      () -> allianceSelection.getAlliance() == Alliance.Red)
    );

    // Hold right button on left joystick to enable fine control for driving
    left[2].onTrue(runOnce(() -> driveTrain.setFineControl(true)));
    left[2].onFalse(runOnce(() -> driveTrain.setFineControl(false)));

    // Outtake coral with end effector when left button on right joystick is pressed
    right[1].onTrue(new EndEffectorCoralOuttake(endEffector, log));
  }

  /**
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // ex: coP[1].onTrue(new command);
  }

  private void configureTriggers() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(driveTrain, log);
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    driveTrain.stopMotors();                // SAFETY:  Turn off any closed loop control that may be running, so the robot does not move when re-enabled.
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    elevator.stopElevatorMotors();

    matchTimer.stop();
    SignalLogger.stop();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");

    driveTrain.setDriveModeCoast(false);
    driveTrain.setVisionForOdometryState(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    matchTimer.reset();
    matchTimer.start();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {

  }
}
