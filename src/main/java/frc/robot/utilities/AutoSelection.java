package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int NONE = 0;
	public static final int choreoTest1 = 1;
	public static final int choreoTest2 = 2;
	public static final int choreoTest3 = 3;
	public static final int circleAuto = 4;
	public static final int rotate180 = 5;

	private final AllianceSelection allianceSelection;
	private final TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, FileLog log) {
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;

		// auto selections
		autoChooser.setDefaultOption("None", NONE);
		autoChooser.addOption("ChoreoTest1", choreoTest1);
		autoChooser.addOption("ChoreoTest2", choreoTest2);
		autoChooser.addOption("ChoreoTest3", choreoTest3);
		autoChooser.addOption("circleAuto", circleAuto);
		autoChooser.addOption("Rotate180", rotate180);


		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		// SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * 
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param log        The filelog to write the logs to
	 * @return the command to run
	 */

	public Command getAutoCommand(DriveTrain driveTrain, FileLog log) {
		Command autonomousCommandMain = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();
		log.writeLogEcho(true, "AutoSelect", "autoPlan",autoPlan);

		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == NONE) {
			// Starting position = facing drivers
			log.writeLogEcho(true, "AutoSelect", "run None");
			autonomousCommandMain = new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain, log);
		}

		else if(autoPlan == choreoTest1){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Test Turning path");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryType.TestPath1), driveTrain, allianceSelection, log);
		}

		else if(autoPlan == choreoTest2){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Test curve path");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryType.TestPath2), driveTrain, allianceSelection, log);
		}

		else if(autoPlan == choreoTest3){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Test Straight path");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryType.TestPath3), driveTrain, allianceSelection, log);
		}

		else if(autoPlan == circleAuto){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Test Circle path");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryType.CirclePath), driveTrain, allianceSelection, log);
		}

		else if(autoPlan == rotate180){
			log.writeLogEcho(true, "AutoSelect", "run Choreo Test Rotate180 path");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryType.Rotate180), driveTrain, allianceSelection, log);
		}

		else if (autonomousCommandMain == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommandMain = new WaitCommand(1);
		}


		// Add auto wait time before the main auto command
		Command autonomousCommand = new SequentialCommandGroup(
			new WaitCommand(waitTime),
			autonomousCommandMain
		);

		return autonomousCommand;
	}

}