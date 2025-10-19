package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.StringUtil;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraWrapper extends SubsystemBase {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimatorCamera;
  private AprilTagFieldLayout aprilTagFieldLayout;
  
  private boolean hasInit = false;
  private int logRotationKey;
  private boolean fastLogging = false;
  private Transform3d robotToCam;
  private String cameraName;

  // Create Data Log Entries
  private final DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> dLogEstPose2D;
  private final BooleanLogEntry dLogConnected = new BooleanLogEntry(log, StringUtil.buildString("/PhotonCameraWrapper/", cameraName, "/Connected") );
  private final BooleanLogEntry dLogTagPresent = new BooleanLogEntry(log, StringUtil.buildString("/PhotonCameraWrapper/", cameraName, "/TagPresent") );
  private boolean priorConnected, priorTagPresent;


  public PhotonCameraWrapper(Transform3d robotToCam, String cameraName, int logRotationKey) {
    
    this.logRotationKey = logRotationKey;
    this.robotToCam = robotToCam;
    this.cameraName = cameraName;

    dLogEstPose2D = StructLogEntry.create(log, StringUtil.buildString("/PhotonCameraWrapper/", cameraName, "/estPose2d"), Pose2d.struct);
  }

  /**
   * Turns file logging on every scheduler cycle (~20 ms) or every 10 cycles (~0.2 sec).
   * @param enabled true = log every cycle, false = log every 10 cycles
   */ 
  public void enableFastLogging(boolean enabled) {
    this.fastLogging = enabled;
  }

  public void init() {
    DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Init, Starting");

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); 
      DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Init, Loaded april tags from file");
    } catch (IOException e) {
      DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Init, Error loading april tags from file");
      e.printStackTrace();
    }

    // Create pose estimator
    photonPoseEstimatorCamera = new PhotonPoseEstimator(
      aprilTagFieldLayout,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
      robotToCam);

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(cameraName);
    }

    hasInit = true;

    // Prime the DataLog to reduce delay when first enabling the robot
    long timeNow = RobotController.getFPGATime();
    dLogEstPose2D.append(Pose2d.kZero, timeNow);
    priorConnected = photonCamera.isConnected();
    dLogConnected.append(priorConnected, timeNow);
    SmartDashboard.putBoolean(StringUtil.buildString("PhotonVision ", cameraName, " Connected"), priorConnected);
    priorTagPresent = false;
    dLogTagPresent.append(priorTagPresent, timeNow);

    DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Init, Done");
  }

  public boolean hasInit() {
    return hasInit;
  }

  public void periodic() {
    // if (fastLogging || DataLogUtil.isMyLogRotation(logRotationKey)) {
      // DataLogUtil.writeMessage("PhotonCameraWrapper (", cameraName, "): Periodic");
    // }
  }

  public String getCameraName() {
    return cameraName;
  }

  /**
   * Gets a PhotonPipelineResult with the best target in Camera Pipeline result and the Camera enum. 
   * If there are no new targets, this method will return null. The best target is determined by the 
   * target sort mode in the PhotonVision UI.
   * @return the best target of the pipeline result
   */
  PhotonPipelineResult getLatestResult() {
    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    return results.isEmpty() ? null : results.get(results.size()-1);
  }

  /**
   * Gets the estimated global pose of the robot on the field.
   * @param prevEstimatedRobotPose the current best guess at robot pose
   * @param latestResult the latest result from the photon vision pipeline
   * @return Optional of the fused camera observations to a single Pose2d on the field, and the time of
   * the observation. Assumes a planar field and the robot is always firmly on the ground.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, PhotonPipelineResult latestResult) {
    long timeNow = RobotController.getFPGATime();

    boolean isConnected = photonCamera.isConnected();
    if (isConnected!=priorConnected) {
      // only write to datalog if this value has changed (saves space in the log)
      dLogConnected.append(isConnected, timeNow);
      SmartDashboard.putBoolean(StringUtil.buildString("PhotonVision ", cameraName, " Connected"), isConnected);
      priorConnected = isConnected;
    }

    photonPoseEstimatorCamera.setReferencePose(prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> newPoseOptional = photonPoseEstimatorCamera.update(latestResult);
    boolean tagPresent = newPoseOptional.isPresent();
    if (tagPresent!=priorTagPresent) {
      // only write to datalog if this value has changed (saves space in the log)
      dLogTagPresent.append(tagPresent, timeNow);
      priorTagPresent = tagPresent;
    }
    if (tagPresent) {
      EstimatedRobotPose newPose = newPoseOptional.get();
      dLogEstPose2D.append(newPose.estimatedPose.toPose2d(), timeNow);
    }
    return newPoseOptional;
  }
}