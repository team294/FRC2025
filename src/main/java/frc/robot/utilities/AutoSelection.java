package frc.robot.utilities;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

		DriveForwardOneMeter("DriveForwardOneMeter", 10),
		BargeToE("BargeToE", 107), // TODO remove test autos
		Rel4mRotate180("Relative4m-180", 101),
		RelArcLeft("RelativeArcLeft", 102),
		RelStraight4m("RelativeStraight4m", 103),
		RelCirclePath("RelativeCirclePath", 104),
		RelRotate180("RelativeRotate180", 105),
		AbsDiagonalTest("AbsoluteDiagonalTest", 106),

		DriveForwardTwoMeters("DriveForwardTwoMeters", 1),

		BargeRight_EDC("BargeRight_EDC", 2),
		BargeLeft_JKL("BargeLeft_JKL", 3),

		BargeRight_ED_AlgaeCD("BargeRight_ED_AlgaeCD", 4),
		BargeLeft_JK_AlgaeKL("BargeLeft_JK_AlgaeKL", 5),
		
		PushFriend_JK("PushFriend_JK", 6),
		PushFriend_JK_AlgaeKL("PushFriend_JK_AlgaeKL", 7),
		
		AutoCenterL1("AutoCenterL1", 8),
		AutoCenterL4("AutoCenterL4", 9);


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
	

	private SendableChooser<Integer> autoRoutineChooser = new SendableChooser<>();
	// private SendableChooser<Integer> startPositionChooser = new SendableChooser<>();

	private static final Map<ReefLocation, Trajectory<SwerveSample>> reefToHPMap = new HashMap<>();
	private static final Map<ReefLocation, Trajectory<SwerveSample>> hpToReefMap = new HashMap<>();
	private static final Map<ReefLocation, Trajectory<SwerveSample>> bargeToReefMap = new HashMap<>();

	/**
	 * Autoselection constructor for command group to set up autoPlan widget.
	 * @param trajectoryCache TrajectoryCache cache
	 * @param allianceSelection AllianceSelection alliance
	 * @param log FileLog log
	 */
	public AutoSelection(Joystick rightJoystick, TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, Field field) {
		this.rightJoystick = rightJoystick;
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;
		this.field = field;
		

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
			// bargeToReefMap.put(ReefLocation.G, null);
			// bargeToReefMap.put(ReefLocation.H, null);
			// bargeToReefMap.put(ReefLocation.I, null);
			bargeToReefMap.put(ReefLocation.J, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE), "BargeRightToJ"));
			bargeToReefMap.put(ReefLocation.K, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToD), "BargeRightToK"));
			bargeToReefMap.put(ReefLocation.L, TrajectoryCache.mirrorVertically(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToC), "BargeRightToL"));
		}

		// Auto selections
		autoRoutineChooser.setDefaultOption(RoutineSelectionOption.NONE.name, RoutineSelectionOption.NONE.value);

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
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * @param driveTrain DriveTrain subsystem
	 * @param elevator Elevator subsystem
	 * @param coralEffector CoralEffector subsystem
	 * @return the command to run
	 */
	public Command getAutoCommand(DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper) {
		Command autonomousCommandMain = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoRoutineChooser.getSelected();
		DataLogUtil.writeMessage(true, "AutoSelect, autoPlan =", autoPlan);

		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15); // make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == RoutineSelectionOption.NONE.value) {
			// Starting position = facing drivers
			DataLogUtil.writeMessage(true, "AutoSelect, run None");
			autonomousCommandMain = new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain);
		}

		else if (autoPlan == RoutineSelectionOption.DriveForwardOneMeter.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run DriveForwardOneMeter");
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
										new DriveToPose(CoordType.kRelative, new Pose2d(1, 0, new Rotation2d(0)), driveTrain));
		}

		else if (autoPlan == RoutineSelectionOption.DriveForwardTwoMeters.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run DriveForwardTwoMeters");
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(allianceSelection.getAlliance() == Alliance.Red ? 0 : 180, false, driveTrain),
										new DriveToPose(CoordType.kRelative, new Pose2d(2, 0, new Rotation2d(0)), driveTrain));
		}

		else if (autoPlan == RoutineSelectionOption.BargeRight_EDC.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run BargeRight_EDC");
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.E, ReefLocation.D, ReefLocation.C));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L3, ReefLevel.L3, ReefLevel.L3));
			autonomousCommandMain = new AutoCoralCycleLoop(reefLocations, reefLevels, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.BargeLeft_JKL.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run BargeLeft_JKL");
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K, ReefLocation.L));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L3, ReefLevel.L3, ReefLevel.L3));
			autonomousCommandMain = new AutoCoralCycleLoop(reefLocations, reefLevels, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.BargeRight_ED_AlgaeCD.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run BargeRight_ED_AlgaeCD");
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.E, ReefLocation.D));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L3, ReefLevel.L3));
			autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.BargeLeft_JK_AlgaeKL.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run BargeLeft_JK_AlgaeKL");
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L3, ReefLevel.L3));
			autonomousCommandMain = new AutoCoralCycleLoopThenAlgae(reefLocations, reefLevels, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.PushFriend_JK.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run PushFriend_JK");
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L3, ReefLevel.L3));
			autonomousCommandMain = new AutoPushFriendThenCoralCycle(reefLocations, reefLevels, true, false, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.PushFriend_JK_AlgaeKL.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run PushFriend_JK_AlgaeKL");
			List<ReefLocation> reefLocations = new ArrayList<>(Arrays.asList(ReefLocation.J, ReefLocation.K));
			List<ReefLevel> reefLevels = new ArrayList<>(Arrays.asList(ReefLevel.L3, ReefLevel.L3));
			autonomousCommandMain = new AutoPushFriendThenCoralCycle(reefLocations, reefLevels, false, true, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, allianceSelection, field);
		}

		else if (autoPlan == RoutineSelectionOption.AutoCenterL1.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run AutoCenterL1");
			autonomousCommandMain = new AutoCenterL1(driveTrain, elevator, wrist, coralEffector, algaeGrabber, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.AutoCenterL4.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run AutoCenterL4");
			autonomousCommandMain = new AutoCenterL4(driveTrain, elevator, wrist, coralEffector, algaeGrabber, field, rightJoystick, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.Rel4mRotate180.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run Rel4mRotate180");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.Relative4mRotate180), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelArcLeft.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run RelArcLeft");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeArcLeft), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelStraight4m.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run RelStraight4m");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeStraight4m), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelCirclePath.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run RelCirclePath");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeCirclePath), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.RelRotate180.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run RelRotate180");
			autonomousCommandMain = new DriveTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.RelativeRotate180), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.AbsDiagonalTest.value) {
			DataLogUtil.writeMessage(true, "AutoSelect, run AbsDiagonalTest");
			autonomousCommandMain = new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.AbsoluteDiagonalTest), driveTrain, allianceSelection);
		}

		else if (autoPlan == RoutineSelectionOption.BargeToE.value) { // test trajectory
			DataLogUtil.writeMessage(true, "AutoSelect, run BargeToE");
			autonomousCommandMain = new SequentialCommandGroup(
										new DriveResetPose(trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE).getInitialPose(allianceSelection.getAlliance() == Alliance.Red).get(), true, driveTrain),
										new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.getTrajectory(TrajectoryName.BargeRightToE), driveTrain, allianceSelection)
									);
		}

		else {
			DataLogUtil.writeMessage(true, "AutoSelect, No autocommand found");
			autonomousCommandMain = new WaitCommand(1);
		}

		// Add auto wait time before the main auto command
		Command autonomousCommand = new SequentialCommandGroup(
			new WaitCommand(waitTime),
			autonomousCommandMain
		);

		return autonomousCommand;
	}

	/**
	 * Schedules the auto command based upon input from the shuffleboard.
	 * This method is designed to be placed on Shuffleboard, so that
	 * the user can run the currently-selected auto command.
	 * @param driveTrain DriveTrain subsystem
	 * @param elevator Elevator subsystem
	 * @param coralEffector CoralEffector subsystem
	 * @return the command to run
	 */
	public Command scheduleAutoCommand(DriveTrain driveTrain, Elevator elevator, Wrist wrist, CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper) {
		return new InstantCommand(
		  () -> {
				Command autonomousCommand = getAutoCommand(driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper);
				if (autonomousCommand != null) {
					autonomousCommand.schedule();
				}
		  }
		);
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