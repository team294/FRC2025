// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class Field {
  private final HashMap<String, Pose2d> reefScoringPositions;
  private final HashMap<String, Pose2d> reefScoringPositionsByAprilTag;
  private final ArrayList<Pose2d> reefScoringPositionListByAprilTag;
  private final HashMap<Integer, Pose2d> reefAprilTagPositions;
  private final ArrayList<Pose2d> reefAprilTagPositionList;
  private final HashMap<Integer, Pose2d> loadingStationAprilTagPositions;
  private final ArrayList<Pose2d> loadingStationAprilTagPositionList;

  private AllianceSelection allianceSelection;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final FileLog log;

  /**
   * Creates a field object that can provide various field locations.
   * All field locations are Pose2d objects based on the current alliance that is selected.
   * Pose components include:
   * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
   * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
   * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
   * @param alliance AllianceSelection utility
   * @param log FileLog utility
   */
  public Field(AllianceSelection allianceSelection, FileLog log){
    this.allianceSelection = allianceSelection;
    this.log = log;

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
      aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      log.writeLog(true, "Field", "Constructor", "Loaded AprilTags from file");
    } catch (Exception exception) {
      log.writeLog(true, "Field", "Constructor", "Error loading AprilTags from file");
      exception.printStackTrace();
    }

    // Reef Scoring Positions, points are on the walls of the reef (Positions are relative, with the origin being the corner to the driver's right)
    // 0 is the closest left scoring position to the drivers, following positions moving clockwise
    // Angles represent the direction the robot would be facing (if scoring mechanism is on the front) when scoring at that point
    // (Rotation of 0 faces away from drivers)
    // Drivers | Field 
    //         |              /  \
    //         |             K    J
    //         |           /        \
    //         |          L           I
    //         |        /              \
    //         |       |                |
    //         |       A                H
    //         |       |      Reef      |
    //         |       B                G
    //         |       |                |
    //         |        \              /
    //         |          C           F
    //         |           \        /
    //         |             D     E
    //         |              \  /
    reefScoringPositions = new HashMap<String, Pose2d>(12);
    reefScoringPositions.put("A", new Pose2d(Units.inchesToMeters(144.0),         Units.inchesToMeters(166.006493),     new Rotation2d(0)           ));
    reefScoringPositions.put("B", new Pose2d(Units.inchesToMeters(144.0),         Units.inchesToMeters(153.292837),     new Rotation2d(0)           ));
    reefScoringPositions.put("C", new Pose2d(Units.inchesToMeters(154.867597965), Units.inchesToMeters(134.469605169),  new Rotation2d(Math.PI/3.0)       ));
    reefScoringPositions.put("D", new Pose2d(Units.inchesToMeters(165.877947035), Units.inchesToMeters(128.112777169),  new Rotation2d(Math.PI/3.0)       ));
    reefScoringPositions.put("E", new Pose2d(Units.inchesToMeters(187.613142965), Units.inchesToMeters(128.112777169),  new Rotation2d(Math.PI*(2.0/3.0)) ));
    reefScoringPositions.put("F", new Pose2d(Units.inchesToMeters(198.623492035), Units.inchesToMeters(134.469605169),  new Rotation2d(Math.PI*(2.0/3.0)) ));
    reefScoringPositions.put("G", new Pose2d(Units.inchesToMeters(209.49109),     Units.inchesToMeters(153.292837),     new Rotation2d(Math.PI)           ));
    reefScoringPositions.put("H", new Pose2d(Units.inchesToMeters(209.49109),     Units.inchesToMeters(166.006493),     new Rotation2d(Math.PI)           ));
    reefScoringPositions.put("I", new Pose2d(Units.inchesToMeters(198.623492035), Units.inchesToMeters(184.829724831),  new Rotation2d(-Math.PI*(2.0/3.0))));
    reefScoringPositions.put("J", new Pose2d(Units.inchesToMeters(187.613142965), Units.inchesToMeters(191.186552831),  new Rotation2d(-Math.PI*(2.0/3.0))));
    reefScoringPositions.put("K", new Pose2d(Units.inchesToMeters(165.877947035), Units.inchesToMeters(191.186552831),  new Rotation2d(-Math.PI/3.0)      ));
    reefScoringPositions.put("L", new Pose2d(Units.inchesToMeters(154.867597965), Units.inchesToMeters(184.829724831),  new Rotation2d(-Math.PI/3.0)      ));

    // AprilTag Pose2d rotated by 180 degrees to face positions into reef
    // Offset along reef edge from april tag to "scoring position" is 0.164331496063 meters
    reefScoringPositionsByAprilTag = new HashMap<String, Pose2d>(12);
    reefScoringPositionsByAprilTag.put("A", aprilTagFieldLayout.getTagPose(18).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(18).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset,  new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("B", aprilTagFieldLayout.getTagPose(18).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(18).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, -FieldConstants.ReefScoringPositionAprilTagOffset, new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("C", aprilTagFieldLayout.getTagPose(17).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(17).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset,  new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("D", aprilTagFieldLayout.getTagPose(17).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(17).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, -FieldConstants.ReefScoringPositionAprilTagOffset, new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("E", aprilTagFieldLayout.getTagPose(22).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(22).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset,  new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("F", aprilTagFieldLayout.getTagPose(22).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(22).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, -FieldConstants.ReefScoringPositionAprilTagOffset, new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("G", aprilTagFieldLayout.getTagPose(21).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(21).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset,  new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("H", aprilTagFieldLayout.getTagPose(21).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(21).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, -FieldConstants.ReefScoringPositionAprilTagOffset, new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("I", aprilTagFieldLayout.getTagPose(20).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(20).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset,  new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("J", aprilTagFieldLayout.getTagPose(20).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(20).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, -FieldConstants.ReefScoringPositionAprilTagOffset, new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("K", aprilTagFieldLayout.getTagPose(19).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(19).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset,  new Rotation2d(0))));
    reefScoringPositionsByAprilTag.put("L", aprilTagFieldLayout.getTagPose(19).get().toPose2d().rotateAround(aprilTagFieldLayout.getTagPose(19).get().toPose2d().getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, -FieldConstants.ReefScoringPositionAprilTagOffset, new Rotation2d(0))));
    reefScoringPositionListByAprilTag = new ArrayList<Pose2d>(reefScoringPositionsByAprilTag.values());

    reefAprilTagPositions = new HashMap<Integer, Pose2d>(18);
    if (aprilTagFieldLayout.getTagPose(1).isEmpty()) {
      reefAprilTagPositions.put(6,  new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), new Rotation2d(Math.toRadians(300)))); // Red K-L
      reefAprilTagPositions.put(7,  new Pose2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), new Rotation2d(Math.toRadians(0)  ))); // Red A-B
      reefAprilTagPositions.put(8,  new Pose2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), new Rotation2d(Math.toRadians(60) ))); // Red C-D
      reefAprilTagPositions.put(9,  new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), new Rotation2d(Math.toRadians(120)))); // Red E-F
      reefAprilTagPositions.put(10, new Pose2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), new Rotation2d(Math.toRadians(180)))); // Red G-H
      reefAprilTagPositions.put(11, new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), new Rotation2d(Math.toRadians(240)))); // Red I-J
      reefAprilTagPositions.put(17, new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), new Rotation2d(Math.toRadians(240)))); // Blue C-D
      reefAprilTagPositions.put(18, new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), new Rotation2d(Math.toRadians(180)))); // Blue A-B
      reefAprilTagPositions.put(19, new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), new Rotation2d(Math.toRadians(120)))); // Blue K-L
      reefAprilTagPositions.put(20, new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), new Rotation2d(Math.toRadians(60) ))); // Blue I-J
      reefAprilTagPositions.put(21, new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), new Rotation2d(Math.toRadians(0)  ))); // Blue G-H
      reefAprilTagPositions.put(22, new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), new Rotation2d(Math.toRadians(300)))); // Blue E-F
    } else {
      // Rotation is with ramp facing reef
      reefAprilTagPositions.put(6,  aprilTagFieldLayout.getTagPose(6).get().toPose2d() ); // Red K-L
      reefAprilTagPositions.put(7,  aprilTagFieldLayout.getTagPose(7).get().toPose2d() ); // Red A-B
      reefAprilTagPositions.put(8,  aprilTagFieldLayout.getTagPose(8).get().toPose2d() ); // Red C-D
      reefAprilTagPositions.put(9,  aprilTagFieldLayout.getTagPose(9).get().toPose2d() ); // Red E-F
      reefAprilTagPositions.put(10, aprilTagFieldLayout.getTagPose(10).get().toPose2d()); // Red G-H
      reefAprilTagPositions.put(11, aprilTagFieldLayout.getTagPose(11).get().toPose2d()); // Red I-J
      reefAprilTagPositions.put(17, aprilTagFieldLayout.getTagPose(17).get().toPose2d()); // Blue C-D
      reefAprilTagPositions.put(18, aprilTagFieldLayout.getTagPose(18).get().toPose2d()); // Blue A-B
      reefAprilTagPositions.put(19, aprilTagFieldLayout.getTagPose(19).get().toPose2d()); // Blue K-L
      reefAprilTagPositions.put(20, aprilTagFieldLayout.getTagPose(20).get().toPose2d()); // Blue I-J
      reefAprilTagPositions.put(21, aprilTagFieldLayout.getTagPose(21).get().toPose2d()); // Blue G-H
      reefAprilTagPositions.put(22, aprilTagFieldLayout.getTagPose(22).get().toPose2d()); // Blue E-F
    }
    reefAprilTagPositionList = new ArrayList<Pose2d>(reefAprilTagPositions.values());

    // Loading stations, side is relative to driver station
    loadingStationAprilTagPositions = new HashMap<Integer, Pose2d>(4);
    loadingStationAprilTagPositions.put(1,  aprilTagFieldLayout.getTagPose(1).get().toPose2d() ); // red left
    loadingStationAprilTagPositions.put(2,  aprilTagFieldLayout.getTagPose(2).get().toPose2d() ); // red right
    loadingStationAprilTagPositions.put(12, aprilTagFieldLayout.getTagPose(12).get().toPose2d()); // blue right
    loadingStationAprilTagPositions.put(13, aprilTagFieldLayout.getTagPose(13).get().toPose2d()); // blue left
    loadingStationAprilTagPositionList = new ArrayList<Pose2d>(loadingStationAprilTagPositions.values());
  }

  /**
   * Flips the x, y, and rotation coordinates across the field, converting between field positions for the red and blue alliances.
   * @param position the Pose2d position to be flipped
   * @return the flipped Pose2d
   */
  public static Pose2d flipPosition(Pose2d position) {
    return new Pose2d(Math.abs(FieldConstants.length - position.getX()), Math.abs(FieldConstants.width - position.getY()), position.getRotation().rotateBy(Rotation2d.kPi));
  }

  /**
   * Gets the Pose2d of the given scoring position. This position is flipped depending on the alliance.
   * @param position letter value associated with one of the 12 scoring positions (A-K starting at the upper 9 o'clock position and moving CCW)
   * @return Pose2d of one of the scoring positions (against the base).
   */
  public Pose2d getReefScoringPosition(String position) {
    return allianceSelection.getAlliance() == Alliance.Blue ? reefScoringPositionsByAprilTag.get(position) : flipPosition(reefScoringPositionsByAprilTag.get(position));
  }

  /**
   * Finds and returns the nearest reef scoring position (against the base).
   * @param currPos the robot's current positions
   * @return Pose2d of the nearest scoring position
   */
  public Pose2d getNearestReefScoringPosition(Pose2d currPos) {
    return getNearestReefScoringPositionWithOffset(currPos, new Transform2d(0, 0, Rotation2d.kZero));
  }

  /**
   * Finds and returns the nearest reef scoring position (against the base) with an offset.
   * @param currPos the robot's current positions
   * @param offset the offset by which the returned pose2d is tranformed
   * @return Pose2d of the nearest scoring position
   */
  public Pose2d getNearestReefScoringPositionWithOffset(Pose2d currPos, Transform2d offset) {
    Pose2d currPosBlue = (allianceSelection.getAlliance() == Alliance.Blue) ? currPos : flipPosition(currPos);
    Pose2d nearestBluePos = currPosBlue.nearest(reefScoringPositionListByAprilTag);
    Pose2d nearestBluePosWithOffset = nearestBluePos.transformBy(offset);
    // log.writeLog(false, "Field", "getNearestReefScoringPositionWithOffset",
    //     "Current X", currPosBlue.getX(),
    //     "Current Y", currPosBlue.getY(),
    //     "Current Rotation", currPosBlue.getRotation().getDegrees(),
    //     "Nearest X", nearestBluePos.getX(),
    //     "Nearest Y", nearestBluePos.getY(),
    //     "Nearest Rotation", nearestBluePos.getRotation().getDegrees(),
    //     "Offset X", offset.getX(),
    //     "Offset Y", offset.getY(),
    //     "Offset Rotation", offset.getRotation().getDegrees(),
    //     "Nearest with Offset X", nearestBluePosWithOffset.getX(),
    //     "Nearest with Offset Y", nearestBluePosWithOffset.getY(),
    //     "Nearest with Offset Rotation", nearestBluePosWithOffset.getRotation().getDegrees()
    // );
    return (allianceSelection.getAlliance() == Alliance.Blue) ? nearestBluePosWithOffset : flipPosition(nearestBluePosWithOffset);
  }

  /**
   * Finds and returns the nearest reef scoring position (against the base)
   * @param currPos the robot's current positions
   * @param left whether the robot should drive to the left (relative to the reef wall) scoring position
   * @return The Pose2d of the nearest scoring position
   */
  public Pose2d getNearestReefScoringPosition(Pose2d currPos, boolean left) {
    return getNearestReefScoringPositionWithOffset(currPos, new Transform2d(0, 0, Rotation2d.kZero), left);
  }

  /**
   * Finds and returns the nearest reef scoring position (against the base) with an offset
   * @param currPos the robot's current positions
   * @param offset the offset by which the returned pose2d is tranformed
   * @param left whether the robot should drive to the left (relative to the reef wall) scoring position
   * @return The Pose2d of the nearest scoring position
   */
  public Pose2d getNearestReefScoringPositionWithOffset(Pose2d currPos, Transform2d offset, boolean left) {
      Pose2d nearestReefAprilTag = getNearestAprilTagReef(currPos);
      Pose2d nearestReefScoringPosition = nearestReefAprilTag.rotateAround(nearestReefAprilTag.getTranslation(), new Rotation2d(Math.PI)).transformBy(new Transform2d(0, FieldConstants.ReefScoringPositionAprilTagOffset * (left ? 1.0 : -1.0),  new Rotation2d(0)));
      Pose2d nearestReefScoringPositionWithOffset = nearestReefScoringPosition.transformBy(offset);
      // log.writeLog(false, "Field", "getNearestReefScoringPositionWithOffset", 
      //     "Current X", currPos.getX(),
      //     "Current Y", currPos.getY(),
      //     "Current Rotation", currPos.getRotation().getDegrees(),
      //     "Nearest April X", nearestReefAprilTag.getX(),
      //     "Nearest April Y", nearestReefAprilTag.getY(),
      //     "Nearest April Rotation", nearestReefAprilTag.getRotation().getDegrees(),
      //     "Left Scoring Position?", left,
      //     "Nearest Scoring X", nearestReefScoringPosition.getX(),
      //     "Nearest Scoring Y", nearestReefScoringPosition.getY(),
      //     "Nearest Scoring Rotation", nearestReefScoringPosition.getRotation().getDegrees(),
      //     "Offset X", offset.getX(),
      //     "Offset Y", offset.getY(),
      //     "Offset Rotation", offset.getRotation().getDegrees(),
      //     "Nearest with Offset X", nearestReefScoringPositionWithOffset.getX(),
      //     "Nearest with Offset Y", nearestReefScoringPositionWithOffset.getY(),
      //     "Nearest with Offset Rotation", nearestReefScoringPositionWithOffset.getRotation().getDegrees()
      // );
      return nearestReefScoringPositionWithOffset;
  }

  /**
   * Gets the Pose2d of the given *Reef* AprilTag.
   * @param tagNumber number value associated with the desired AprilTag
   * @return Pose2d of the given AprilTag
   */
  public Pose2d getReefScoringPosition(int tagNumber) {
    return reefAprilTagPositions.get(tagNumber);
  }

  /**
   * Finds and returns the nearest *Reef* AprilTag position with an offset
   * @param currPos the robot's current positions
   * @return The Pose2d of the nearest reef AprilTag
   */
  public Pose2d getNearestAprilTagReef(Pose2d currPos) {
      return getNearestAprilTagReefWithOffset(currPos, new Transform2d(0, 0, Rotation2d.kZero));
  }

  /**
   * Finds and returns the nearest *Reef* AprilTag position
   * @param currPos the robot's current positions
   * @param offset the offset by which the returned pose2d is tranformed
   * @return The Pose2d of the nearest reef AprilTag
   */
  public Pose2d getNearestAprilTagReefWithOffset(Pose2d currPos, Transform2d offset) {
      Pose2d nearestReefAprilTag = currPos.nearest(reefAprilTagPositionList);
      Pose2d nearestReefAprilTagWithOffset = nearestReefAprilTag.transformBy(offset);
      // log.writeLog(false, "Field", "getNearestAprilTagWithOffset", 
      //     "Current X", currPos.getX(),
      //     "Current Y", currPos.getY(),
      //     "Current Rotation", currPos.getRotation().getDegrees(),
      //     "Nearest X", nearestReefAprilTag.getX(),
      //     "Nearest Y", nearestReefAprilTag.getY(),
      //     "Nearest Rotation", nearestReefAprilTag.getRotation().getDegrees(),
      //     "Offset X", offset.getX(),
      //     "Offset Y", offset.getY(),
      //     "Offset Rotation", offset.getRotation().getDegrees(),
      //     "Nearest with Offset X", nearestReefAprilTagWithOffset.getX(),
      //     "Nearest with Offset Y", nearestReefAprilTagWithOffset.getY(),
      //     "Nearest with Offset Rotation", nearestReefAprilTagWithOffset.getRotation().getDegrees()
      // );
      return nearestReefAprilTagWithOffset;
  }

  /**
   * Finds and returns the nearest *Loading Station* aprilTag position.
   * @param currPos the robot's current position
   * @return Pose2d of the nearest loading station AprilTag
   */
  public Pose2d getNearestAprilTagLoadingStation(Pose2d currPos) {
    Pose2d nearestAprilTagLoadingStation = currPos.nearest(loadingStationAprilTagPositionList);
    // log.writeLog(false, "Field", "getNearestAprilTagLoadingStation", 
    //     "Current X", currPos.getX(),
    //     "Current Y", currPos.getY(),
    //     "Current Rotation", currPos.getRotation().getDegrees(),
    //     "Nearest X", nearestAprilTagLoadingStation.getX(),
    //     "Nearest Y", nearestAprilTagLoadingStation.getY(),
    //     "Nearest Rotation", nearestAprilTagLoadingStation.getRotation().getDegrees()
    // );
    return nearestAprilTagLoadingStation;
  }
}
