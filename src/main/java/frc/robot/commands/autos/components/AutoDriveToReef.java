package frc.robot.commands.autos.components;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.Constants.ElevatorWristConstants.*;
import frc.robot.commands.*;
import frc.robot.commands.sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.ElevatorWristRegions.RegionType;

public class AutoDriveToReef extends SequentialCommandGroup {
  /** Below constructor is outdated and not used anywhere
   * Drives to reef location (using trajectory) and prepares elevator based on the reef level.
   * Runs intake command while driving to ensure coral is fully in, and then move the elevator after done driving.
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param trajectoryName TrajectoryName name of trajectory to follow 
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection alliance 
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */ /* 
  public AutoDriveToReef(ReefLevel level, TrajectoryName trajectoryName, DriveTrain driveTrain, Elevator elevator, Wrist wrist,
        CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance, TrajectoryCache cache, FileLog log) {
    addCommands(
      new DataLogMessage(false, "AutoDriveToReefAndPrep", "Init", "trajectoryName", trajectoryName.toString()),
      parallel(
        new CoralIntakeSequence(elevator, wrist, hopper, coralEffector),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.getTrajectory(trajectoryName), driveTrain, alliance)
      ),
      new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist)
      // TODO add DriveToPose here, check these parameters
      // new DriveToPose(CoordType.kRelative, () -> new Pose2d(DriveConstants.driveBackFromReefDistance, 0, Rotation2d.kZero), 
      //         0.5, 1.0, 
      //         TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      //         true, true, driveTrain)
    );
  } */

  /**
   * Drives to reef location, <b> trajectory gets cut off early (commented out) </b> to allow for smooth transition into AutomatedDriveToReefAndScoreCoral (does not move elevator)
   * Runs intake command while driving to ensure coral is fully in, and then move the elevator after done driving.
   * @param fromHP true = starting at HP, false = starting at barge
   * @param end ReefLocation (A-L) to drive to
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector CoralEffector subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection alliance 
   * @param log FileLog log
   */
  public AutoDriveToReef(boolean fromHP, ReefLocation end, DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
      CoralEffector coralEffector, Hopper hopper, AllianceSelection alliance) {

    // Choose trajectory to drive (Start from either HP or barge)
    Trajectory<SwerveSample> trajectory = fromHP ? AutoSelection.getHPToReef(end) : AutoSelection.getBargeToReef(end);
    addCommands(
      new DataLogMessage(false, "AutoDriveToReef: Start, trajectory =", trajectory.name()),

      deadline(
        // Drives the trajectory while intaking coral to make sure coral is being intaked. Timeout in case coral doesn't make it into hopper.
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectory, driveTrain, alliance).withTimeout(trajectory.getTotalTime() - 0.5),
        sequence(
          new CoralIntakeSequence(elevator, wrist, hopper, coralEffector).withTimeout(3),
          new WristElevatorSafeMove(ElevatorWristPosition.CORAL_L1, RegionType.CORAL_ONLY, elevator, wrist)
        )
      ),

      // If the endeffector is not in hold mode(Coral is not safely in intake) then do nothing, otherwise intake the coral until the piece is present
      either(
        none(), 
        new CoralIntakeSequence(elevator, wrist, hopper, coralEffector), 
        () -> (coralEffector.getHoldMode())),
      
      new DataLogMessage(false, "AutoDriveToReef: End, trajectory =", trajectory.name())



      //new WristElevatorSafeMove(reefToElevatorMap.get(level), RegionType.CORAL_ONLY, elevator, wrist, log)

      // Drive to the given reef position 
      // new DriveToPose(CoordType.kAbsolute, () -> (field.getRobotReefScoringPosition(end)), 0.5, 1.0, TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, true, true, driveTrain, log)
    );
  }
}