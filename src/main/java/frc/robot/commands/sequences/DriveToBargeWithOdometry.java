// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class DriveToBargeWithOdometry extends SequentialCommandGroup {
  /**
   * Drives to the nearest barge scoring location.
   *
   * <p>If the robot is aligned with the correct alliance barge, the robot will drive foward towards
   * the barge while maintaining its current y value.
   *
   * <p>If the robot is not aligned with the correct alliance barge, the robot will drive foward
   * while driving to the y value of the edge of the barge.
   *
   * <p>The sequence ends when robot reaches final position. If the driver continues to hold the
   * button, the robot can only move left to right.
   *
   * @param driveTrain DriveTrain subsystem
   * @param field Field utility
   */
  public DriveToBargeWithOdometry(DriveTrain driveTrain, Field field) {
    addCommands(
        new DataLogMessage(false, "DriveToBargeWithOdometry: Start"),
        parallel(
                new DriveToPose(
                    CoordType.kAbsolute,
                    () -> (field.getNearestBargeScoringPosition(driveTrain.getPose())),
                    0.02,
                    1,
                    driveTrain),
                runOnce(
                    () -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.AUTOMATED_DRIVING_BARGE)))
            .handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
        new DataLogMessage(false, "DriveToBargeWithOdometry: End"));
  }
}
