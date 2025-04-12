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
import frc.robot.commands.autos.components.*;
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
  private final AllianceSelection allianceSelection = new AllianceSelection();
  private final Field field = new Field(allianceSelection);
  private final Timer matchTimer = new Timer();

  // Define robot subsystems
  private final LED led = new LED(Ports.CANdle, "LED", matchTimer);
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection);
  private final Hopper hopper = new Hopper("Hopper");
  private final Wrist wrist = new Wrist("Wrist");
  private final CoralEffector coralEffector = new CoralEffector("CoralEffector", wrist);
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber("AlgaeGrabber");
  private final Elevator elevator = new Elevator("Elevator");
  // private final Climber climber = new Climber("Climber");

  // Define controllers
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);
  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache();
  // private final AutoSelection autoSelection = new AutoSelection(rightJoystick, trajectoryCache, allianceSelection, field);
  private final AutoSelection autoSelection = new AutoSelection(rightJoystick, trajectoryCache, allianceSelection, 
      field, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper);

  // Define commands
  private final CoralIntakeSequence coralIntakeSequence = new CoralIntakeSequence(elevator, wrist, hopper, coralEffector);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DataLogUtil.writeLogEcho(true, "RobotContainer", "Constructor", "Version", Constants.bcrRobotCodeVersion);
    SignalLogger.enableAutoLogging(false);
    
    // Start LEDEventUtil
    LEDEventUtil.start(led);

    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences");
    }

    // Set the Choreo trajectory flipper to the 2025 field orientation.
    // When the alliance is Red, Choreo will automatically flip Blue trajectories to Red.
    // All trajectories should be defined based on the Blue alliance perspective.
    ChoreoAllianceFlipUtil.setYear(2025);

    configureButtonBindings();
    configureShuffleboard();

    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvanced(leftJoystick, rightJoystick, allianceSelection, driveTrain, field));
    // driveTrain.setDefaultCommand(new DriveWithJoysticks(leftJoystick, rightJoystick, allianceSelection, driveTrain));
    // driveTrain.setDefaultCommand(new DriveWithController(driveTrain, xboxController, allianceSelection));

    // Set initial robot position on field. This takes place a while after the drivetrain is created, so after any CANbus delays.
    // Set initial location to (0,0) and facing away from driver (regardless of alliance color).
    driveTrain.resetPose(new Pose2d(0.0, 0.0, 
      allianceSelection.getAlliance() == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero));
  }

  private void configureShuffleboard() {
    // Sticky Faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear());

    // DriveTrain
    SmartDashboard.putData("Drive FOC On", new DriveSetFOC(true, driveTrain));
    SmartDashboard.putData("Drive FOC Off", new DriveSetFOC(false, driveTrain));
    SmartDashboard.putData("Drive Toggle Coast", new DriveToggleCoastMode(driveTrain));
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain));
    SmartDashboard.putData("Drive To Pose", new DriveToPose(CoordType.kAbsolute, driveTrain));
    SmartDashboard.putData("Drive 6m Fwd", new DriveToPose(CoordType.kRelative, () -> new Pose2d(6.0, 0.0, new Rotation2d(0.0)),
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      true, true, driveTrain));
    SmartDashboard.putData("Drive Calibration", new DriveCalibration(0.0, 0.5, 5.0, 0.1, driveTrain));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain));
    SmartDashboard.putData("Drive Percent Speed", new DrivePercentSpeed(driveTrain));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain));

    // Hopper
    SmartDashboard.putData("Hopper Forward", new HopperSetPercent(0.4, hopper));
    SmartDashboard.putData("Hopper Reverse", new HopperSetPercent(-0.4, hopper));
    SmartDashboard.putData("Hopper STOP", new HopperStop(hopper));

    // CoralEffector
    SmartDashboard.putData("CoralEffector Forward", new CoralEffectorSetPercent(0.05, coralEffector));
    SmartDashboard.putData("CoralEffector Reverse", new CoralEffectorSetPercent(-0.05, coralEffector));
    SmartDashboard.putData("CoralEffector STOP", new CoralEffectorStop(coralEffector));
    SmartDashboard.putData("CoralEffector Intake", new CoralEffectorIntake(coralEffector));
    SmartDashboard.putData("CoralEffector Outtake", new CoralEffectorOuttake(coralEffector));
    SmartDashboard.putData("CoralEffector Intake Enhanced", new CoralEffectorIntakeEnhanced(coralEffector));
    SmartDashboard.putData("CoralEffector Set Percent", new CoralEffectorSetPercent(coralEffector));
    SmartDashboard.putData("CoralEffector Set Position", new CoralEffectorSetPosition(false, coralEffector));

    // AlgaeGrabber
    SmartDashboard.putData("AlgaeGrabber In", new AlgaeGrabberSetPercent(0.1, algaeGrabber));
    SmartDashboard.putData("AlgaeGrabber Out", new AlgaeGrabberSetPercent(-0.1, algaeGrabber));
    SmartDashboard.putData("AlgaeGrabber STOP", new AlgaeGrabberStop(algaeGrabber));
    SmartDashboard.putData("AlgaeGrabber Intake", new AlgaeGrabberIntake(algaeGrabber));
    SmartDashboard.putData("AlgaeGrabber Outtake", new AlgaeGrabberOuttake(algaeGrabber));

    // Wrist
    SmartDashboard.putData("Wrist STOP", new WristStop(wrist));
    SmartDashboard.putData("Wrist Up", new WristSetPercent(WristConstants.maxManualPercentOutput, wrist));
    SmartDashboard.putData("Wrist Down", new WristSetPercent(-WristConstants.maxManualPercentOutput, wrist));
    SmartDashboard.putData("Wrist Set Percent", new WristSetPercent(wrist));
    SmartDashboard.putData("Wrist Set Angle", new WristSetAngle(wrist));
    SmartDashboard.putData("Wrist Cal. to START CONFIG", new WristCalibrateManual(ElevatorWristPosition.START_CONFIG.wristAngle, wrist));
    SmartDashboard.putData("Wrist Calibrate Ramp", new WristCalibrationRamp(0.01, 0.2, wrist));

    // Elevator
    SmartDashboard.putData("Elevator STOP", new ElevatorStop(elevator));
    SmartDashboard.putData("Elevator Up", new ElevatorSetPercent(ElevatorConstants.maxManualPercentOutput, false, elevator));
    SmartDashboard.putData("Elevator Down", new ElevatorSetPercent(-ElevatorConstants.maxManualPercentOutput, false, elevator));
    SmartDashboard.putData("Elevator Calibration Routine", new ElevatorCalibration(0.1, elevator));
    SmartDashboard.putData("Elevator Set Position", new ElevatorSetPosition(elevator));

    // Climber
    // SmartDashboard.putData("Climber STOP", new ClimberStop(climber));
    // SmartDashboard.putData("Climber Set Percent", new ClimberSetPercentOutput(climber));
    // SmartDashboard.putData("Climber Set Angle", new ClimberSetAngle(climber));
    // SmartDashboard.putData("Climber Cal. to START CONFIG", new ClimberCalibrateManual(ClimberConstants.ClimberAngle.CALIBRATE_MANUAL.value, climber));
    // SmartDashboard.putData("Climber Run Calibration", new ClimberCalibrationRamp(-0.05, 0.25, climber));
    
    // Autos
    SmartDashboard.putData("Autonomous Run Auto Now", autoSelection.scheduleAutoCommand());
    SmartDashboard.putData("Auto Barge Right To E", new AutoCoralDriveAndScoreSequence(false, ReefLocation.E, ReefLevel.L1, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field));
    SmartDashboard.putData("Auto E to HP", new AutoCoralDriveAndIntakeSequence(ReefLocation.E, driveTrain, elevator, wrist, coralEffector, hopper, allianceSelection));
    SmartDashboard.putData("Auto HP to E", new AutoCoralDriveAndScoreSequence(true, ReefLocation.E, ReefLevel.L1, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field));

    // Copanel buttons

    // Vision
    SmartDashboard.putData("Vision Enable Odometry Updates", new VisionOdometryStateSet(true, driveTrain));
    SmartDashboard.putData("Vision Disable Odometry Updates", new VisionOdometryStateSet(false, driveTrain));

    // Sequences
    SmartDashboard.putData("Algae Intake Sequence Ground", new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_GROUND, elevator, wrist, algaeGrabber));
    SmartDashboard.putData("Algae Intake Sequence Lower", new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_LOWER, elevator, wrist, algaeGrabber));
    SmartDashboard.putData("Algae Intake Sequence Upper", new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_UPPER, elevator, wrist, algaeGrabber));
    SmartDashboard.putData("Algae Score Prep Sequence Processor", new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_PROCESSOR, elevator, wrist, algaeGrabber));
    SmartDashboard.putData("Algae Score Prep Sequence Net", new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_NET, elevator, wrist, algaeGrabber));

    SmartDashboard.putData("Coral Intake Sequence", new CoralIntakeSequence(elevator, wrist, hopper, coralEffector));
    SmartDashboard.putData("Coral Score Prep Sequence L1", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L1, elevator, wrist, algaeGrabber, coralEffector));
    SmartDashboard.putData("Coral Score Prep Sequence L2", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L2, elevator, wrist, algaeGrabber, coralEffector));
    SmartDashboard.putData("Coral Score Prep Sequence L3", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L3, elevator, wrist, algaeGrabber, coralEffector));
    SmartDashboard.putData("Coral Score Prep Sequence L4", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L4, elevator, wrist, algaeGrabber, coralEffector));
    SmartDashboard.putData("Coral Score Prep Sequence L4-Copanel", new CoralScorePrepSequence(ElevatorWristPosition.CORAL_L4_COPANEL, elevator, wrist, algaeGrabber, coralEffector));

    SmartDashboard.putData("Stow Elevator and Wrist", new WristElevatorSafeMove(ElevatorWristPosition.CORAL_HP, RegionType.STANDARD, elevator, wrist));

    SmartDashboard.putData("AutomatedDriveToReefAndIntakeAlgae", new AutomatedDriveToReefAndIntakeAlgae(driveTrain, elevator, wrist, algaeGrabber, field));
    // SmartDashboard.putData("Climber Prep Sequence", new ClimberPrepSequence(elevator, wrist, climber));
    // SmartDashboard.putData("Climber Set Angle to Lift", new ClimberSetAngle(ClimberConstants.ClimberAngle.CLIMB_END, climber));

    // Stop All Motors
    SmartDashboard.putData("Stop All Motors", parallel(
      new HopperStop(hopper),
      new AlgaeGrabberStop(algaeGrabber),
      new CoralEffectorStop(coralEffector),
      new WristStop(wrist),
      new ElevatorStop(elevator)
    ));
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
    xbRT.onTrue(new CoralIntakeSequence(elevator, wrist, hopper, coralEffector));

    // xbX, A, B, and Y will automatically drive to the reef and score coral, see right[1] in configureJoystickButtons()

    // Prep and intake algae from Ground with LT, Reef Lower with D-Pad Down, and Reef Upper with D-Pad Left
    xbLT.onTrue(either(
        new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_GROUND, elevator, wrist, algaeGrabber),
        none(),
        () -> !coralEffector.isCoralPresent()
      )
    );
    //xbPOVDown.onTrue(new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_LOWER, driveTrain, elevator, wrist, algaeGrabber, led));  //Replaced with AutomatedDriveToReefAndIntakeAlgae in conjuction with rigth joystick button
    //xbPOVLeft.onTrue(new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_UPPER, driveTrain, elevator, wrist, algaeGrabber, led));

    // Prep and intake algae from Lollipop with Back
    xbBack.onTrue(either(
      new AlgaeIntakeSequence(ElevatorWristPosition.ALGAE_LOLLIPOP, elevator, wrist, algaeGrabber),
      none(),
      () -> !coralEffector.isCoralPresent()
    ));

    // Prep to score algae in Net with D-Pad Up and Processor with D-Pad Right
    xbPOVUp.onTrue(new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_NET,elevator, wrist, algaeGrabber));
    xbPOVRight.onTrue(new AlgaeScorePrepSequence(ElevatorWristPosition.ALGAE_PROCESSOR, elevator, wrist, algaeGrabber));

    // Manually control elevator with right joystick
    xbRJoystickTrigger.whileTrue(new ElevatorManualControl(xboxController, elevator, true));

    // Manually control wrist with the left joystick
    xbLJoystickTrigger.whileTrue(new WristManualControl(xboxController, wrist, false));

    // Stop all motors with LB
    xbLB.onTrue(parallel(
      new HopperStop(hopper),
      new AlgaeGrabberStop(algaeGrabber),
      new CoralEffectorStop(coralEffector),
      new WristStop(wrist),
      new ElevatorStop(elevator)
    ));

    // Move wrist and elevator to home
    xbRB.onTrue(new WristElevatorSafeMove(ElevatorWristPosition.START_CONFIG, RegionType.CORAL_ONLY, elevator, wrist));
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

    left[1].onTrue(new AlgaeGrabberOuttake(algaeGrabber));
    left[2].onTrue(new CoralEffectorOuttake(coralEffector));

    right[1].and(xboxController.x()).whileTrue(new AutomatedDriveToReefAndScoreCoral(ReefLevel.L1, driveTrain, elevator, wrist, coralEffector, algaeGrabber, rightJoystick, field));
    right[1].and(xboxController.a()).whileTrue(new AutomatedDriveToReefAndScoreCoral(ReefLevel.L2, driveTrain, elevator, wrist, coralEffector, algaeGrabber, rightJoystick, field));
    right[1].and(xboxController.b()).whileTrue(new AutomatedDriveToReefAndScoreCoral(ReefLevel.L3, driveTrain, elevator, wrist, coralEffector, algaeGrabber, rightJoystick, field));
    right[1].and(xboxController.y()).whileTrue(new AutomatedDriveToReefAndScoreCoral(ReefLevel.L4, driveTrain, elevator, wrist, coralEffector, algaeGrabber, rightJoystick, field));

    right[1].and(xboxController.povLeft()).whileTrue(new AutomatedDriveToReefAndIntakeAlgae(ElevatorWristPosition.ALGAE_UPPER, driveTrain, elevator, wrist, algaeGrabber, field));
    right[1].and(xboxController.povDown()).whileTrue(new AutomatedDriveToReefAndIntakeAlgae(ElevatorWristPosition.ALGAE_LOWER, driveTrain, elevator, wrist, algaeGrabber, field));

    right[2].whileTrue(new DriveToBargeWithOdometry(driveTrain, field));
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
    
    // Elevator and Wrist Commands
    coP[1].onTrue(new CoralScorePrepSequence(ElevatorWristConstants.ElevatorWristPosition.CORAL_L4_COPANEL, elevator, wrist, algaeGrabber, coralEffector));
    coP[2].onTrue(new CoralScorePrepSequence(ElevatorWristConstants.ElevatorWristPosition.CORAL_L3, elevator, wrist, algaeGrabber, coralEffector));
    coP[9].onTrue(new CoralScorePrepSequence(ElevatorWristConstants.ElevatorWristPosition.CORAL_L2, elevator, wrist, algaeGrabber, coralEffector));
    coP[10].onTrue(new CoralScorePrepSequence(ElevatorWristConstants.ElevatorWristPosition.CORAL_L1, elevator, wrist, algaeGrabber, coralEffector));

    // Elevator Commands
    coP[3].whileTrue(new ElevatorSetPercent(ElevatorConstants.maxManualPercentOutput, false, elevator));
    coP[3].onFalse(new ElevatorStop(elevator));
    coP[4].onTrue(new ElevatorSetPercent(-ElevatorConstants.maxManualPercentOutput, false, elevator));
    coP[4].onFalse(new ElevatorStop(elevator));

    // Wrist Commands
    coP[11].whileTrue(new WristSetPercent(WristConstants.maxManualPercentOutput, wrist));
    coP[12].whileTrue(new WristSetPercent(-WristConstants.maxManualPercentOutput, wrist));

    // Hopper Commands
    coP[5].onTrue(new HopperSetPercent(-HopperConstants.reverseIntakePercent, hopper));
    coP[5].onFalse(new HopperStop(hopper));
    coP[6].onTrue(new HopperSetPercent(HopperConstants.reverseIntakePercent, hopper));
    coP[6].onFalse(new HopperStop(hopper));

    // Coral Commands
    coP[13].onTrue(new CoralEffectorSetPercent(CoralEffectorConstants.intakePercent, coralEffector));
    coP[13].onFalse(new CoralEffectorStop(coralEffector));
    coP[14].onTrue(new CoralEffectorSetPercent(-CoralEffectorConstants.intakePercent, coralEffector));
    coP[14].onFalse(new CoralEffectorStop(coralEffector));

    // Reset Pose
    coP[7].onTrue(either(
      new DriveResetPose(180, false, driveTrain), 
      new DriveResetPose(0, false, driveTrain), 
      () -> allianceSelection.getAlliance() == Alliance.Red));

    // Release Game Piece Commands
    coP[15].onTrue(new CoralEffectorOuttake(coralEffector));
    coP[16].onTrue(new AlgaeGrabberOuttake(algaeGrabber));

    // Manual Calibration Commands
    coP[17].onTrue(new ElevatorCalibrateIfAtLowerLimit(elevator));
    coP[18].onTrue(new WristCalibrateManual(ElevatorWristConstants.ElevatorWristPosition.START_CONFIG.wristAngle, wrist));

    // Algae Grabber Commands
    coP[19].onTrue(new AlgaeGrabberSetPercent(AlgaeGrabberConstants.intakePercent, algaeGrabber));
    coP[19].onFalse(new AlgaeGrabberStop(algaeGrabber));
    coP[20].onTrue(new AlgaeGrabberSetPercent(AlgaeGrabberConstants.netOuttakePercent, algaeGrabber));
    coP[20].onFalse(new AlgaeGrabberStop(algaeGrabber));
  }

  private void configureTriggers() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand();
  }

  /**
   * Method called once every scheduler cycle (20 ms).
   */
  public void robotPeriodic(){
    DataLogUtil.advanceLogRotation();
    allianceSelection.periodic();
    autoSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    // Do not log the word "Init" here, as it it affects the Excel macro
    DataLogUtil.writeLogEcho(true, "Disabled", "Robot disabled");

    driveTrain.stopMotors();             // SAFETY: Turn off any closed loop control that may be running, so the robot does not move when re-enabled
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    elevator.stopElevatorMotors();
    wrist.stopWrist();
    coralEffector.stopCoralEffectorMotor();

    LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL);
    LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.ROBOT_DISABLED);

    matchTimer.stop();
    matchTimer.reset();
    SignalLogger.stop();
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error (to prevent the issue that caused us to be eliminated in 2020!)
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus");
    }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    DataLogUtil.writeLogEcho(true, "Auto", "Mode Init");

    driveTrain.setDriveModeCoast(false);
    driveTrain.setVisionForOdometryState(true);

    // NOTE: Do not reset the gyro or encoder here!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.

    coralEffector.stopCoralEffectorMotor();

    if (elevator.isElevatorCalibrated()) {
      elevator.setElevatorProfileTarget(elevator.getElevatorPosition());
    } else {
      elevator.stopElevatorMotors();
    }
    if (wrist.isWristCalibrated()) {
      wrist.setWristAngle(wrist.getWristAngle());
    } else {
      wrist.stopWrist();
    }
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
    DataLogUtil.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false); // Set drive mode to brake mode
    driveTrain.enableFastLogging(false); // Turn off fast logging, in case it was left on from auto mode
    driveTrain.setVisionForOdometryState(true);

    coralEffector.stopCoralEffectorMotor();
    coralIntakeSequence.schedule();

    if (elevator.isElevatorCalibrated()) {
      elevator.setElevatorProfileTarget(elevator.getElevatorPosition());
    } else {
      elevator.stopElevatorMotors();
    }
    if (wrist.isWristCalibrated()) {
      wrist.setWristAngle(wrist.getWristAngle());
    } else {
      wrist.stopWrist();
    }

    LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL);
    
    matchTimer.reset();
    matchTimer.start();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
  }
}
