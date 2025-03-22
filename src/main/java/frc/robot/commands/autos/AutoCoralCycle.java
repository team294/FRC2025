// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.autos.components.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoCoralCycle extends SequentialCommandGroup {
  /**
   * Based on a start (reef location) and end (reef location to score at), run a cycle that:
   *   - Drives from reef to HP and intakes a coral
   *   - Drives to reef and scores the coral
   * @param start ReefLocation (A-L) to start at
   * @param end ReefLocation (A-L) to end at
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param alliance AllianceSelection alliance
   * @param cache TrajectoryCache cache
   * @param log FileLog log
   */
  public AutoCoralCycle(ReefLocation start, ReefLocation end, ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist,
          CoralEffector coralEffector, Hopper hopper, Joystick rightJoystick, AllianceSelection alliance, Field field, FileLog log) {
    addCommands(
      // Drives from start reef location to HP and intakes coral  
      new AutoCoralDriveAndIntakeSequence(start, driveTrain, elevator, wrist, coralEffector, hopper, alliance, log),
      // Drives from HP to end reef location and scores coral
      new AutoCoralDriveAndScoreSequence(true, end, level, driveTrain, elevator, wrist, coralEffector, hopper, rightJoystick, alliance, field, log)
    );
  }
}
