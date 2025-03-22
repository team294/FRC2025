// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.MathBCR;

public class ChoreoFollower extends Command {
  private Timer timer = new Timer();
  private Trajectory<SwerveSample> trajectory;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private Consumer<ChassisSpeeds> outputChassisSpeed;
  private Supplier<Pose2d> poseSupplier;
  private BooleanSupplier mirrorTrajectory;
  private boolean mirrorTrajectoryThisInit;
  private DriveTrain driveTrain;
  private FileLog log;
  private StructLogEntry<Pose2d> pose2DEntry;
  private DoubleLogEntry trajXEntry, trajYEntry;

  /**
   * Choreo follower used to follow Choreo trajectories. 
   * This method takes in a choreo trajectory and follows the contents of that trajectory until the trajectory is finished.
   * <p><b>NOTE:</b> This follower will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user to do since it is not appropriate for paths with nonstationary endstates.
   * @param trajectory Choreo trajectory to follow
   * @param xController trajectory PID X feedback controller
   * @param yController trajectory PID Y feedback controller 
   * @param rotationController rotation PID feedback controller
   * @param outputChassisSpeeds a function that takes in a field-relative chassis speeds and controls the robot
   * @param poseSupplier a pose supplier to get the current robot position
   * @param mirrorTrajectory determines if the trajectory will be mirrored: when true the trajectory will mirror while 
   * internal robot odometry will remain the same, when false the trajectory will not be changed
   * @param driveTrain DriveTrain subsystem
   * @param log LogFile utility
   */
  public ChoreoFollower(Trajectory<SwerveSample> trajectory, PIDController xController, PIDController yController,
      PIDController rotationController, Consumer<ChassisSpeeds> outputChassisSpeeds,
      Supplier<Pose2d> poseSupplier, BooleanSupplier mirrorTrajectory, DriveTrain driveTrain, FileLog log) {
    this.trajectory = trajectory;
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;
    this.outputChassisSpeed = outputChassisSpeeds;
    this.poseSupplier = poseSupplier;
    this.mirrorTrajectory = mirrorTrajectory;
    this.driveTrain = driveTrain;
    this.log = log;

    DataLog logD = DataLogManager.getLog();
    pose2DEntry = StructLogEntry.create(logD, "/ChoreoFollower/curPose2d", Pose2d.struct);
    trajXEntry = new DoubleLogEntry(logD, "/ChoreoFollower/trajX");
    trajYEntry = new DoubleLogEntry(logD, "/ChoreoFollower/trajY");

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveTrain.enableFastLogging(true);
    
    timer.reset();
    timer.start();

    xController.reset();
    yController.reset();
    rotationController.reset();

    mirrorTrajectoryThisInit = mirrorTrajectory.getAsBoolean();

    log.writeLog(false, "ChoreoFollower", "Init", "Is Flipped", mirrorTrajectoryThisInit);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xFF;
    double yFF;
    double rotationFF;
    double xFeedback ;
    double yFeedback;
    double rotationFeedback;

    double curTime = timer.get();

    Optional<SwerveSample> sampleCurrOpt = trajectory.sampleAt(curTime, mirrorTrajectoryThisInit);
    Optional<SwerveSample> sampleNextOpt = trajectory.sampleAt(curTime + Constants.SwerveConstants.dt, mirrorTrajectoryThisInit);

    if(!sampleCurrOpt.isEmpty() && !sampleNextOpt.isEmpty()){
      SwerveSample sampleCurr = sampleCurrOpt.get();
      SwerveSample sampleNext = sampleNextOpt.get();
      Pose2d poseCurr = poseSupplier.get();

      xFF = sampleNext.vx;
      yFF = sampleNext.vy;
      rotationFF = sampleNext.omega;
      xFeedback = xController.calculate(poseCurr.getX(), sampleCurr.x);
      yFeedback = yController.calculate(poseCurr.getY(), sampleCurr.y);
      rotationFeedback = rotationController.calculate(poseCurr.getRotation().getRadians(), sampleCurr.heading);

      long timeNow = RobotController.getFPGATime();
      pose2DEntry.append(poseCurr, timeNow);
      trajXEntry.append(sampleCurr.x, timeNow);
      trajYEntry.append(sampleCurr.y, timeNow);
      

      ChassisSpeeds robotSpeeds = driveTrain.getRobotSpeeds();
      log.writeLog(false, "ChoreoFollower", "Execute", 
        "Time", curTime,
        "Traj X", sampleCurr.x,
        "Traj Y", sampleCurr.y,
        "Traj Vel", Math.hypot(sampleCurr.vx, sampleCurr.vy),
        "Traj VelAng", Math.toDegrees(Math.atan2(sampleCurr.vy, sampleCurr.vx)),
        "Traj rot", Math.toDegrees(sampleCurr.heading),
        "Traj rotVel", Math.toDegrees(sampleCurr.omega),
        "Robot X", poseCurr.getX(),
        "Robot Y", poseCurr.getY(),
        "Robot Vel", Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond),
        "Robot VelAng", Math.toDegrees(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond)),
        "Robot rot", poseCurr.getRotation().getDegrees(),
        "Robot rotVel", driveTrain.getAngularVelocity(),
        driveTrain.getDriveModuleOutputs()
      );
        
    } else {
      xFF = 0;
      yFF = 0;
      rotationFF = 0;
      xFeedback = 0;
      yFeedback = 0;
      rotationFeedback = 0;
    }
    
    outputChassisSpeed.accept(new ChassisSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback)); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveTrain.enableFastLogging(false);
    timer.stop();
    log.writeLog(false, "ChoreoFollower", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Optional<Pose2d> finalPoseOpt = trajectory.getFinalPose(mirrorTrajectoryThisInit);

    if (finalPoseOpt.isEmpty()){
      log.writeLog(false, "ChoreoFollower", "IsFinished", "No final Pose exists");
      return true;
    }

    Pose2d finalPose = finalPoseOpt.get();
    Pose2d poseCurr = poseSupplier.get();

    return (timer.hasElapsed(trajectory.getTotalTime())
        && Math.abs(finalPose.getX() - poseCurr.getX()) <= Constants.TrajectoryConstants.xError
        && Math.abs(finalPose.getY() - poseCurr.getY()) <= Constants.TrajectoryConstants.yError
        && Math.abs(MathBCR.normalizeAngle(poseCurr.getRotation().getDegrees()
            - finalPose.getRotation().getDegrees())) <= Constants.TrajectoryConstants.choreoMaxThetaErrorDegrees
        && Math.hypot(driveTrain.getChassisSpeeds().vxMetersPerSecond,
            driveTrain.getChassisSpeeds().vyMetersPerSecond) <= Constants.TrajectoryConstants.endVelocityErrorDegrees)
        ||
        timer.hasElapsed(trajectory.getTotalTime() + 0.7);
  }
}
