// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;
import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.Constants.FieldConstants.ReefLocation;

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
  // private final LED led = new LED(Constants.Ports.CANdle, "LED", matchTimer, log);
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection, log);
  private final Hopper hopper = new Hopper("Hopper", log);
  private final CoralEffector coralEffector = new CoralEffector("CoralEffector", log);
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber("AlgaeGrabber", log);
  private final Wrist wrist = new Wrist("Wrist", log);
  private final Elevator elevator = new Elevator("Elevator", log);
  private final Climber climber = new Climber("Climber", log);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, allianceSelection, log);

  // Define controllers
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);
  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }

    // Set the Choreo trajectory flipper to the 2025 field orientation.
    // When the alliance is Red, Choreo will automatically flip Blue trajectories to Red.
    // All trajectories should be defined based on the Blue alliance perspective.
    ChoreoAllianceFlipUtil.setYear(2025);

    configureButtonBindings();
    configureShuffleboard();

    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvanced(leftJoystick, rightJoystick, allianceSelection, driveTrain, field, log));
    // driveTrain.setDefaultCommand(new DriveWithJoysticks(leftJoystick, rightJoystick, allianceSelection, driveTrain, log));
    // driveTrain.setDefaultCommand(new DriveWithController(driveTrain, xboxController, allianceSelection, log));

    // Set initial robot position on field. This takes place a while after the drivetrain is created, so after any CANbus delays.
    // Set initial location to (0,0) and facing away from driver (regardless of alliance color).
    driveTrain.resetPose(new Pose2d(0.0, 0.0, 
      allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero));
  }

  private void configureShuffleboard() {
    // Sticky Faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));

    // DriveTrain
    SmartDashboard.putData("Drive FOC On", new DriveSetFOC(true, driveTrain, log));
    SmartDashboard.putData("Drive FOC Off", new DriveSetFOC(false, driveTrain, log));
    SmartDashboard.putData("Drive Toggle Coast", new DriveToggleCoastMode(driveTrain, log));
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain, log));
    SmartDashboard.putData("Drive To Pose", new DriveToPose(CoordType.kAbsolute, driveTrain, log));
    SmartDashboard.putData("Drive 6m Fwd", new DriveToPose(CoordType.kRelative, () -> new Pose2d(6.0, 0.0, new Rotation2d(0.0)),
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      true, true, driveTrain, log));
    SmartDashboard.putData("Drive Calibration", new DriveCalibration(0.0, 0.5, 5.0, 0.1, driveTrain, log));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain, log));
    SmartDashboard.putData("Drive Percent Speed", new DrivePercentSpeed(driveTrain, log));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain, log));

    // Hopper
    SmartDashboard.putData("Hopper Forward", new HopperSetPercent(0.4, hopper, log));
    SmartDashboard.putData("Hopper Reverse", new HopperSetPercent(-0.4, hopper, log));
    SmartDashboard.putData("Hopper STOP", new HopperStop(hopper, log));

    // CoralEffector
    SmartDashboard.putData("CoralEffector Forward", new CoralEffectorSetPercent(0.05, coralEffector, log));
    SmartDashboard.putData("CoralEffector Reverse", new CoralEffectorSetPercent(-0.05, coralEffector, log));
    SmartDashboard.putData("CoralEffector STOP", new CoralEffectorStop(coralEffector, log));
    SmartDashboard.putData("CoralEffector Intake", new CoralEffectorIntake(coralEffector, log));
    SmartDashboard.putData("CoralEffector Outtake", new CoralEffectorOuttake(coralEffector, log));

    // AlgaeGrabber
    SmartDashboard.putData("AlgaeGrabber In", new AlgaeGrabberSetPercent(0.1, algaeGrabber, log));
    SmartDashboard.putData("AlgaeGrabber Out", new AlgaeGrabberSetPercent(-0.1, algaeGrabber, log));
    SmartDashboard.putData("AlgaeGrabber STOP", new AlgaeGrabberStop(algaeGrabber, log));
    SmartDashboard.putData("AlgaeGrabber Intake", new AlgaeGrabberIntake(algaeGrabber, log));
    SmartDashboard.putData("AlgaeGrabber Outtake", new AlgaeGrabberOuttake(algaeGrabber, log));

    // Wrist
    SmartDashboard.putData("Wrist STOP", new WristStop(wrist, log));
    SmartDashboard.putData("Wrist Set Percent", new WristSetPercent(wrist, log));
    SmartDashboard.putData("Wrist Set Angle", new WristSetAngle(wrist, log));
    SmartDashboard.putData("Wrist Cal. to START CONFIG", new WristCalibrateManual(ElevatorWristPosition.START_CONFIG.wristAngle, wrist, log));
    SmartDashboard.putData("Wrist Calibrate Ramp", new WristCalibrationRamp(0.01, 0.2, wrist, log));

    // Elevator
    SmartDashboard.putData("Elevator Up", new ElevatorSetPercent(ElevatorConstants.maxManualPercentOutput, false, elevator, log));
    SmartDashboard.putData("Elevator Down", new ElevatorSetPercent(-ElevatorConstants.maxManualPercentOutput, false, elevator, log));
    SmartDashboard.putData("Elevator STOP", new ElevatorStop(elevator, log));
    SmartDashboard.putData("Elevator Move To 20 In", new ElevatorSetPosition(20.0, elevator, log));
    SmartDashboard.putData("Elevator Move To 65 In", new ElevatorSetPosition(65.0, elevator, log));
    SmartDashboard.putData("Elevator Move to HP", new ElevatorSetPosition(ElevatorWristPosition.CORAL_HP, elevator, log));
    SmartDashboard.putData("Elevator Move to L1", new ElevatorSetPosition(ElevatorWristPosition.CORAL_L1, elevator, log));
    SmartDashboard.putData("Elevator Move To L2", new ElevatorSetPosition(ElevatorWristPosition.CORAL_L2, elevator, log));
    SmartDashboard.putData("Elevator Move To L3", new ElevatorSetPosition(ElevatorWristPosition.CORAL_L3, elevator, log));
    SmartDashboard.putData("Elevator Move To L4", new ElevatorSetPosition(ElevatorWristPosition.CORAL_L4, elevator, log));
    SmartDashboard.putData("Elevator Calibration Routine", new ElevatorCalibration(0.1, elevator, log));

    // Climber
    SmartDashboard.putData("Climber STOP", new ClimberStop(climber, log));
    SmartDashboard.putData("Climber Set Percent", new ClimberSetPercentOutput(climber, log));
    SmartDashboard.putData("Climber Set Angle", new ClimberSetAngle(climber, log));
    SmartDashboard.putData("Climber Cal. to START CONFIG", new ClimberCalibrateManual(ClimberConstants.ClimberAngle.CALIBRATE_MANUAL.value, climber, log));
    SmartDashboard.putData("Climber Run Calibration", new ClimberCalibrationRamp(-0.05, 0.25, climber, log));
    
    // Autos
    SmartDashboard.putData("Autonomous Run Auto Now", autoSelection.scheduleAutoCommand(driveTrain, elevator, endEffector));
    SmartDashboard.putData("Auto Barge Right To E", new AutoCoralDriveAndScoreSequence(false, ReefLocation.E, ReefLevel.L1, driveTrain, elevator, endEffector, allianceSelection, trajectoryCache, log));
    SmartDashboard.putData("Auto E to HP", new AutoCoralDriveAndIntakeSequence(ReefLocation.E, driveTrain, elevator, endEffector, allianceSelection, trajectoryCache, log));
    SmartDashboard.putData("Auto HP to E", new AutoCoralDriveAndScoreSequence(true, ReefLocation.E, ReefLevel.L1, driveTrain, elevator, endEffector, allianceSelection, trajectoryCache, log));

    // Copanel buttons

    // Vision
    SmartDashboard.putData("Vision Enable Odometry Updates", new VisionOdometryStateSet(true, driveTrain, log));
    SmartDashboard.putData("Vision Disable Odometry Updates", new VisionOdometryStateSet(false, driveTrain, log));

    // Sequences
    SmartDashboard.putData("Algae Intake Sequence Ground", new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_GROUND, elevator, wrist, algaeGrabber, log));
    SmartDashboard.putData("Algae Intake Sequence Lower", new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_LOWER, elevator, wrist, algaeGrabber, log));
    SmartDashboard.putData("Algae Intake Sequence Upper", new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_UPPER, elevator, wrist, algaeGrabber, log));
    SmartDashboard.putData("Algae Score Prep Sequence Processor", new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_PROCESSOR, elevator, wrist, algaeGrabber, log));
    SmartDashboard.putData("Algae Score Prep Sequence Net", new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_NET, elevator, wrist, algaeGrabber, log));

    SmartDashboard.putData("Coral Intake Sequence", new CoralIntakeSequence(elevator, wrist, hopper, coralEffector, log));
    SmartDashboard.putData("Coral Score Prep Sequence L1", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L1, elevator, wrist, coralEffector, algaeGrabber, log));
    SmartDashboard.putData("Coral Score Prep Sequence L2", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L2, elevator, wrist, coralEffector, algaeGrabber, log));
    SmartDashboard.putData("Coral Score Prep Sequence L3", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L3, elevator, wrist, coralEffector, algaeGrabber, log));
    SmartDashboard.putData("Coral Score Prep Sequence L4", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L4, elevator, wrist, coralEffector, algaeGrabber, log));

    SmartDashboard.putData("Climber Prep Sequence", new ClimberPrepSequence(elevator, wrist, climber, log));
    SmartDashboard.putData("Climber Set Angle to Lift", new ClimberSetAngle(ClimberConstants.ClimberAngle.CLIMB_END, climber, log));
  }
 
   private void configureButtonBindings() {
    configureXboxButtons();     // configure Xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel();         // configure copanel
    configureTriggers();
  }

  /**
   * Define Xbox buttons and controls.
   */
  private void configureXboxButtons() {
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

    // Move elevator and wrist, and run hopper and coralEffector to intake coral with RT
    xbRT.onTrue(new CoralIntakeSequence(elevator, wrist, hopper, coralEffector, log));

    // Move wrist and elevator to home
    xbX.onTrue(new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist, log));

    // Prep to score coral on L2 with A, L3 with B, and L4 with Y
    xbA.onTrue(new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L2, elevator, wrist, coralEffector, algaeGrabber, log));
    xbB.onTrue(new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L3, elevator, wrist, coralEffector, algaeGrabber, log));
    xbY.onTrue(new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L4, elevator, wrist, coralEffector, algaeGrabber, log));

    // Prep and intake algae from Ground with LT, Reef Lower with D-Pad Down, and Reef Upper with D-Pad Left
    xbLT.onTrue(new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_GROUND, elevator, wrist, algaeGrabber, log));
    xbPOVDown.onTrue(new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_LOWER, elevator, wrist, algaeGrabber, log));
    xbPOVLeft.onTrue(new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_UPPER, elevator, wrist, algaeGrabber, log));

    // Prep to score algae in Net with D-Pad Up and Processor with D-Pad Right
    xbPOVUp.onTrue(new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_NET,elevator, wrist, algaeGrabber, log));
    xbPOVRight.onTrue(new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_PROCESSOR, elevator, wrist, algaeGrabber, log));

    // Manually control elevator with right joystick
    xbRJoystickTrigger.whileTrue(new ElevatorManualControl(xboxController, elevator, log, true));

    // Manually control wrist with the left joystick
    xbLJoystickTrigger.whileTrue(new WristManualControl(xboxController, wrist, log, false));

    // Stop all motors with LB
    xbLB.onTrue(parallel(
      new HopperStop(hopper, log),
      new AlgaeGrabberStop(algaeGrabber, log),
      new CoralEffectorStop(coralEffector, log),
      new WristStop(wrist, log),
      new ElevatorStop(elevator, log)
    ));
  }

  /**
   * Define driver joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // ex: left[1].onTrue(new command);

    // 180 if we are red, 0 if we are blue
    left[1].onTrue(
      either(
        new DriveResetPose(180, false, driveTrain, log), 
        new DriveResetPose(0, false, driveTrain, log), 
        () -> allianceSelection.getAlliance() == Alliance.Red
      )
    );
    left[2].onTrue(new ScorePieceSequence(coralEffector, algaeGrabber, driveTrain, log));

    right[1].whileTrue(new DriveToReefWithOdometryForCoral(driveTrain, field, rightJoystick, log));
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
   *  15 17 19
   *  16 18 20
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[21];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // ex: coP[1].onTrue(new command);
    
    coP[1].onTrue(new ElevatorSetPercent(ElevatorConstants.maxManualPercentOutput, false, elevator, log));
    coP[2].onTrue(new ElevatorSetPercent(-ElevatorConstants.maxManualPercentOutput, false, elevator, log));

    coP[3].onTrue(new WristSetPercent(WristConstants.maxManualPercentOutput, wrist, log));
    coP[4].onTrue(new WristSetPercent(-WristConstants.maxManualPercentOutput, wrist, log));

    coP[5].onTrue(new ClimberSetPercentOutput(ClimberConstants.maxManualPercentOutput, climber, log));
    coP[6].onTrue(new ClimberSetPercentOutput(-ClimberConstants.maxManualPercentOutput, climber, log));

    coP[8].onTrue(new ClimberPrepSequence(elevator, wrist, climber, log));
    coP[13].onTrue(new ClimberSetAngle(ClimberConstants.ClimberAngle.CLIMB_END, climber, log));

    coP[9].onTrue(new CoralEffectorOuttake(coralEffector, log));
    coP[10].onTrue(new CoralEffectorIntake(coralEffector, log));

    coP[11].onTrue(new AlgaeGrabberOuttake(algaeGrabber, log));
    coP[12].onTrue(new AlgaeGrabberIntake(algaeGrabber, log));

    // 180 if we are red, 0 if we are blue
    coP[7].onTrue(either(
        new DriveResetPose(180, false, driveTrain, log), 
        new DriveResetPose(0, false, driveTrain, log), 
        () -> allianceSelection.getAlliance() == Alliance.Red));
  }

  private void configureTriggers() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(driveTrain, elevator, coralEffector);
    return none();
  }

  /**
   * Method called once every scheduler cycle (20 ms).
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    // Do not log the word "Init" here, as it it affects the Excel macro
    log.writeLogEcho(true, "Disabled", "Robot disabled");

    driveTrain.stopMotors();             // SAFETY: Turn off any closed loop control that may be running, so the robot does not move when re-enabled
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    // driveTrain.setVisionForOdometryState(true);

    elevator.stopElevatorMotors();
    wrist.stopWrist();

    matchTimer.stop();
    SignalLogger.stop();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error (to prevent the issue that caused us to be eliminated in 2020!)
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
    // driveTrain.setVisionForOdometryState(false);

    // NOTE: Do not reset the gyro or encoder here!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled.
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false); // Set drive mode to brake mode
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    // driveTrain.setVisionForOdometryState(true);

    matchTimer.reset();
    matchTimer.start();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
  }
}
