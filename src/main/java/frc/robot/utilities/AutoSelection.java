package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants.CoordType;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryName;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {
	public enum RoutineSelectionOption {
		NONE("None", -1),

		DriveForwardOneMeter("DriveForwardOneMeter", 0),
		// BargeToE("BargeToE", 107), // TODO remove test autos

		// Rel4mRotate180("Relative4m-180", 101),
		// RelArcLeft("RelativeArcLeft", 102),
		// RelStraight4m("RelativeStraight4m", 103),
		// RelCirclePath("RelativeCirclePath", 104),
		// RelRotate180("RelativeRotate180", 105),
		// AbsDiagonalTest("AbsoluteDiagonalTest", 106),

		DriveForwardTwoMeters("DriveForwardTwoMeters", 1),

		BargeRight_EDC("BargeRight_EDC", 2),
		BargeLeft_JKL("BargeLeft_JKL", 3),

		CenterOneCoralTwoAlgae("CenterOneCoralTwoAlgae", 4),
		CenterOneCoralOneAlgae("CenterOneCoralOneAlgae", 5), 
		CenterOneCoralScoreGHIntakeIJ("CenterOneCoralScoreGHIntakeIJ", 6),

		// BargeRight_ED_AlgaeCD("BargeRight_ED_AlgaeCD", 4),
		// BargeLeft_JK_AlgaeKL("BargeLeft_JK_AlgaeKL", 5),
		
		// PushFriend_JK("PushFriend_JK", 5),
		// PushFriend_JK_AlgaeKL("PushFriend_JK_AlgaeKL", 6),
		
		AutoCenterL1("AutoCenterL1", 7),
		AutoCenterL4("AutoCenterL4", 8);
		
		// BargeRight_EDC_L2("BargeRight_EDC_L2", 9),
		// BargeLeft_JKL_L2("BargeLeft_JKL_L2", 11);


		@SuppressWarnings({ "MemberName", "PMD.SingularField" })
		public final String name;
		public final int value;

		RoutineSelectionOption(String name, int value) {
			this.name = name;
			this.value = value;
		}
	}

	public enum StartPositionSelectionOption {
		RIGHT("Right", 0),
		LEFT("Left", 1);

		@SuppressWarnings({ "MemberName", "PMD.SingularField" })
		public final String name;
		public final int value;

		StartPositionSelectionOption(String name, int value) {
			this.name = name;
			this.value = value;
		}
	}

	private final AllianceSelection allianceSelection;
	private final TrajectoryCache trajectoryCache;
	private final Field field;
	private final Joystick rightJoystick;
	private final DriveTrain driveTrain;
	private final Elevator elevator;
	private final Wrist wrist;
	private final CoralEffector coralEffector;
	private final AlgaeGrabber algaeGrabber;
	private final Hopper hopper;
	private final Climber climber;
	
	private Command cachedAutonomousCommand;
	private int logRotationKey;
	private int autoPlan;
	private double waitTime;
	private Alliance alliance;

	private SendableChooser<Integer> autoRoutineChooser = new SendableChooser<>();
	// private SendableChooser<Integer> startPositionChooser = new SendableChooser<>();

	private static final Map<ReefLocation, Trajectory<SwerveSample>> reefToHPMap = new HashMap<>();
	private static final Map<ReefLocation, Trajectory<SwerveSample>> hpToReefMap = new HashMap<>();
	private static final Map<ReefLocation, Trajectory<SwerveSample>> bargeToReefMap = new HashMap<>();

	/**
	 * Autoselection constructor for command group to set up autoPlan widget.
	 * @param rightJoystick
	 * @param trajectoryCache TrajectoryCache cache
	 * @param allianceSelection AllianceSelection alliance
	 * @param field Field utility
	 * @param driveTrain DriveTrain subsystem
	 * @param elevator Elevator subsystem
	 * @param wrist Wrist subsystem
	 * @param coralEffector Coral Effector subsystem
	 * @param algaeGrabber Algae Grabber subsystem
	 * @param hopper Hopper subsystem
	 * @param climber Climber subsystem
	 */
	public AutoSelection(Joystick rightJoystick, TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field,
						 DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, Climber climber) {
		this.rightJoystick = rightJoystick;
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;
		this.field = field;
		this.driveTrain = driveTrain;
		this.elevator = elevator;
		this.wrist = wrist;
		this.coralEffector = coralEffector;
		this.algaeGrabber = algaeGrabber;
		this.hopper = hopper;
		this.climber = climber;

		logRotationKey = DataLogUtil.allocateLogRotation();

		// initializer to populate the trajectory maps TODO create more trajectories
		// left-side trajectories are the right-side trajectories mirrored over the line going through the middle of the reef 
		// Mirroring:
		//       Right side: B, C, D, E, F, G
		//        Left side: A, L, K, J, I, H
		{
			// Populate reefToHPMap
			// reefToHPMap.put(ReefLocation.A, null);
			// reefToHPMap.put(ReefLocation.B, null);
			reefToHPMap.put(ReefLocation.C, trajectoryCache.getTrajectory(TrajectoryName.CToHP));
			reefToHPMap.put(ReefLocation.D, trajectoryCache.getTrajectory(TrajectoryName.DToHP));
			reefToHPMap.put(ReefLocation.E, trajectoryCache.getTrajectory(TrajectoryName.EToHP));
			// reefToHPMap.put(ReefLocation.F, null);
			// reefToHPMap.put(ReefLocation.G, null);
			// reefToHPMap.put(ReefLocation.H, null);
			// reefToHPMap.put(ReefLocation.I, null);
			reefToHPMap.put(ReefLocation.J, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.EToHP), "JToHP"));
			reefToHPMap.put(ReefLocation.K, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.DToHP), "KToHP"));
			reefToHPMap.put(ReefLocation.L, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.CToHP), "LToHP"));

			// Populate hpToReefMap
			// hpToReefMap.put(ReefLocation.A, null);
			// hpToReefMap.put(ReefLocation.B, null);
			hpToReefMap.put(ReefLocation.C, trajectoryCache.getTrajectory(TrajectoryName.HPToC));
			hpToReefMap.put(ReefLocation.D, trajectoryCache.getTrajectory(TrajectoryName.HPToD));
			hpToReefMap.put(ReefLocation.E, trajectoryCache.getTrajectory(TrajectoryName.HPToE));
			// hpToReefMap.put(ReefLocation.F, null);
			// hpToReefMap.put(ReefLocation.G, null);
			// hpToReefMap.put(ReefLocation.H, null);
			// hpToReefMap.put(ReefLocation.I, null);
			hpToReefMap.put(ReefLocation.J, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.HPToE), "HPToJ"));
			hpToReefMap.put(ReefLocation.K, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.HPToD), "HPToK"));
			hpToReefMap.put(ReefLocation.L, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.HPToC), "HPToL"));

			// Populate bargeToReefMap
			// bargeToReefMap.put(ReefLocation.A, null);
			// bargeToReefMap.put(ReefLocation.B, null);
			bargeToReefMap.put(ReefLocation.C, trajectoryCache.getTrajectory(TrajectoryName.BargeRightToC));
			bargeToReefMap.put(ReefLocation.D, trajectoryCache.getTrajectory(TrajectoryName.BargeRightToD));
			bargeToReefMap.put(ReefLocation.E, trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE));
			// bargeToReefMap.put(ReefLocation.F, null);
			bargeToReefMap.put(ReefLocation.G, trajectoryCache.getTrajectory(TrajectoryName.BargeCenterToG));
			bargeToReefMap.put(ReefLocation.H, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeCenterToG), "BargeCenterToH"));
			// bargeToReefMap.put(ReefLocation.I, null);
			bargeToReefMap.put(ReefLocation.J, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE), "BargeRightToJ"));
			bargeToReefMap.put(ReefLocation.K, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToD), "BargeRightToK"));
			bargeToReefMap.put(ReefLocation.L, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToC), "BargeRightToL"));
		}

		// Auto selections
		autoRoutineChooser.setDefaultOption(RoutineSelectionOption.NONE.name, RoutineSelectionOption.NONE.value);

		// Set initial values
		autoPlan = -999;
		waitTime = 0;
		alliance = allianceSelection.getAlliance();

		for (RoutineSelectionOption option : RoutineSelectionOption.values()){
			if (option == RoutineSelectionOption.NONE) continue;
			autoRoutineChooser.addOption(option.name, option.value);
		}
	
		// Show auto selection widget on dashboard
		SmartDashboard.putData("Autonomous routine", autoRoutineChooser);

		// Show auto parameters on dashboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		// SmartDashboard.putBoolean("Autonomous use vision", false);

		// Show start position selection widget on dashboard
		// SmartDashboard.putData("Start position", startPositionChooser);

		// Build initial auto command
		buildAutoCommand();
	}

	/**
	 * Builds the auto command based upon input from the shuffleboard and caches it to later be
	 * returned by getAutoCommand().
	 * This method is designed to be called in advance of AutonomousInit by Robot.java.
	 */
	public void buildAutoCommand() {
		Command autonomousCommandMain = null;

		// Get parameters from Shuffleboard
		autoPlan = autoRoutineChooser.getSelected();
		RoutineSelectionOption autoSelectionOption = RoutineSelectionOption.NONE;
		DataLogUtil.writeMessage(true, "AutoSelect, autoPlan =", autoPlan);

		waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15); // make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == RoutineSelectionOption.NONE.value) {
			// Starting position = facing drivers
			autoSelectionOption = RoutineSelectionOption.NONE;
			autonomousCommandMain = new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain);
		}

		else if (autoPlan == RoutineSelectionOption.DriveForwardOneMeter.value) {
			autoSelectionOption = RoutineSelectionOption.DriveForwardOneMeter;
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
										new DriveToPose(CoordType.kRelative, new Pose2d(1, 0, new Rotation2d(0)), driveTrain));
		}

		else if (autoPlan == RoutineSelectionOption.DriveForwardTwoMeters.value) {
			autoSelectionOption = RoutineSelectionOption.DriveForwardTwoMeters;
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
										new DriveToPose(CoordType.kRelative, new Pose2d(2, 0, new Rotation2d(0)), driveTrain));
		}

		else if (autoPlan == RoutineSelectionOption.BargeRight_EDC.value) {
			autoSelectionOption = RoutineSelectionOption.BargeRight_EDC;
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.E, ReefLocation.D, ReefLocation.C));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4, ReefLevel.L4, ReefLevel.L4));
			autonomousCommandMain = new AutoCoralCycleLoop(reefLocations, reefLevels, false, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.BargeLeft_JKL.value) {
			autoSelectionOption = RoutineSelectionOption.BargeLeft_JKL;
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K, ReefLocation.L));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4, ReefLevel.L4, ReefLevel.L4));
			autonomousCommandMain = new AutoCoralCycleLoop(reefLocations, reefLevels, false, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field);
		}

		// else if (autoPlan == RoutineSelectionOption.BargeRight_EDC_L2.value) {
		// 	autoSelectionOption = RoutineSelectionOption.BargeRight_EDC_L2;
		// 	List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.E, ReefLocation.D, ReefLocation.C));
		// 	List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L2, ReefLevel.L4, ReefLevel.L4));
		// 	autonomousCommandMain = new AutoCoralCycleLoop(reefLocations, reefLevels, false, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field);
		// }

		// else if (autoPlan == RoutineSelectionOption.BargeLeft_JKL_L2.value) {
		// 	autoSelectionOption = RoutineSelectionOption.BargeLeft_JKL_L2;
		// 	List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K, ReefLocation.L));
		// 	List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L2, ReefLevel.L4, ReefLevel.L4));
		// 	autonomousCommandMain = new AutoCoralCycleLoop(reefLocations, reefLevels, false, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field);
		// }

		// else if (autoPlan == RoutineSelectionOption.BargeRight_ED_AlgaeCD.value) {
		// 	autoSelectionOption = RoutineSelectionOption.BargeRight_ED_AlgaeCD;
		// 	List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.E, ReefLocation.D));
		// 	List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4, ReefLevel.L4));
		// 	autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, led, rightJoystick, allianceSelection, field);
		// }

		// else if (autoPlan == RoutineSelectionOption.BargeLeft_JK_AlgaeKL.value) {
		// 	autoSelectionOption = RoutineSelectionOption.BargeLeft_JK_AlgaeKL;
		// 	List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K));
		// 	List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4, ReefLevel.L4));
		// 	autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, led, rightJoystick, allianceSelection, field);
		// }

		else if (autoPlan == RoutineSelectionOption.CenterOneCoralTwoAlgae.value) {
			autoSelectionOption = RoutineSelectionOption.CenterOneCoralTwoAlgae;
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.H));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4));
			autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, true, true, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field, trajectoryCache);
		}

		else if (autoPlan == RoutineSelectionOption.CenterOneCoralScoreGHIntakeIJ.value) {
			autoSelectionOption = RoutineSelectionOption.CenterOneCoralTwoAlgae;
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.H));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4));
			autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, true, true, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field, trajectoryCache);
		}

		else if (autoPlan == RoutineSelectionOption.CenterOneCoralOneAlgae.value) {
			autoSelectionOption = RoutineSelectionOption.CenterOneCoralOneAlgae;
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.H));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L4));
			autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, true, false, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, climber, rightJoystick, allianceSelection, field, trajectoryCache);
		}

		else if (autoPlan == RoutineSelectionOption.AutoCenterL1.value) {
			autoSelectionOption = RoutineSelectionOption.AutoCenterL1;
			autonomousCommandMain = new AutoCenterL1(driveTrain, elevator, wrist, coralEffector, algaeGrabber, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.AutoCenterL4.value) {
			autoSelectionOption = RoutineSelectionOption.AutoCenterL4;
			autonomousCommandMain = new AutoCenterL4(driveTrain, elevator, wrist, coralEffector, algaeGrabber, field, rightJoystick, allianceSelection);
		}

		// else if (autoPlan == RoutineSelectionOption.Rel4mRotate180.value) {
		// 	autoSelectionOption = RoutineSelectionOption.Rel4mRotate180;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.Relative4mRotate180), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelArcLeft.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelArcLeft;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeArcLeft), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelStraight4m.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelStraight4m;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeStraight4m), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelCirclePath.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelCirclePath;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeCirclePath), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.RelRotate180.value) {
		// 	autoSelectionOption = RoutineSelectionOption.RelRotate180;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeRotate180), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.AbsDiagonalTest.value) {
		// 	autoSelectionOption = RoutineSelectionOption.AbsDiagonalTest;
		// 	autonomousCommandMain = new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.AbsoluteDiagonalTest), driveTrain, allianceSelection);
		// }

		// else if (autoPlan == RoutineSelectionOption.BargeToE.value) { // test trajectory
		// 	autoSelectionOption = RoutineSelectionOption.BargeToE;
		// 	autonomousCommandMain = new SequentialCommandGroup(
		// 								new DriveResetPose(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE).getInitialPose(allianceSelection.getAlliance() == Alliance.Red).get(), true, driveTrain),
		// 								new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE), driveTrain, allianceSelection)
		// 							);
		// }

		else {
			autoSelectionOption = RoutineSelectionOption.NONE;
			autonomousCommandMain = new WaitCommand(1);
		}

		// Add auto wait time before the main auto command
		cachedAutonomousCommand = new SequentialCommandGroup(
			new WaitCommand(waitTime),
			autonomousCommandMain
		);

		String autoDescription = StringUtil.buildString(allianceSelection.getAlliance(), " ", autoSelectionOption);
		SmartDashboard.putString("Autonomous cached", autoDescription);
		DataLogUtil.writeMessage("AutoSelection: Built ", autoDescription);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * @return the command to run
	 */
	public Command getAutoCommand() {
		if (cachedAutonomousCommand == null) {
			buildAutoCommand();
		}
		DataLogUtil.writeMessage("AutoSelection: GetAutoCommand");
		return cachedAutonomousCommand;
	}

	/**
	 * Schedules the auto command based upon input from the shuffleboard.
	 * This method is designed to be placed on Shuffleboard, so that
	 * the user can run the currently-selected auto command.
	 * @return the command to run
	 */
	public Command scheduleAutoCommand() {
		return new InstantCommand(
		  () -> {
				Command autonomousCommand = getAutoCommand();
				if (autonomousCommand != null) {
					autonomousCommand.schedule();
				}
		  }
		);
	}

	/**
	 * Runs once per scheduler cycle.
	 */
	public void periodic() {
		if (DataLogUtil.isMyLogRotation(logRotationKey)) {
			int newAutoPlan = autoRoutineChooser.getSelected();
			double newWaitTime = SmartDashboard.getNumber("Autonomous delay", 0);
			Alliance newAlliance = allianceSelection.getAlliance();

			if ( (newAutoPlan != autoPlan) || (newWaitTime != waitTime) ||
				 (newAlliance != alliance)) {
				alliance = newAlliance;
				// Note that buildAutoCommand will update autoPlan and waitTime
				buildAutoCommand();
			}
		}
	}

	/**
	 * Get the trajectoryName from a reef location to HP
	 * @param start ReefLocation (A-L) start position
	 * @return TrajectoryName name of trajectory
	 */
	public static Trajectory<SwerveSample> getReefToHP(ReefLocation start) {
		return reefToHPMap.get(start);
	}

	/**
	 * Get the trajectoryName from HP to a reef location
	 * @param end ReefLocation (A-L) end position
	 * @return TrajectoryName name of trajectory
	 */
	public static Trajectory<SwerveSample> getHPToReef(ReefLocation end) {
		return hpToReefMap.get(end);
	}

	/**
	 * Get the trajectoryName from barge to a reef location
	 * @param end ReefLocation (A-L) end position%
	 * @return TrajectoryName name of trajectory
	 */
	public static Trajectory<SwerveSample> getBargeToReef(ReefLocation end) {
		return bargeToReefMap.get(end);
	}
}