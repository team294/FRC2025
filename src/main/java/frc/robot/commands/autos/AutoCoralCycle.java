// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants.*;
import frc.robot.commands.DataLogMessage;
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
   * @param dontScore false = we want to move the elevator to score the coral; true = stop at reef DOESN'T WORK
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param hopper Hopper subsystem
   * @param rightJoystick Joystick joystick
   * @param alliance AllianceSelection alliance
   * @param field Field field
   */
  public AutoCoralCycle(ReefLocation start, ReefLocation end, boolean dontScore, ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist,
          CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Hopper hopper, Joystick rightJoystick, AllianceSelection alliance, Field field) {
    addCommands(
      new DataLogMessage(false, "AutoCoralCycle: Start"),
      // Drives from start reef location to HP and intakes coral  
      new AutoCoralDriveAndIntakeSequence(start, driveTrain, elevator, wrist, coralEffector, hopper, alliance),

      // If not the last coral, then drives from HP to reef location and scores coral
      // Otherwise, if it is the last coral, then drives from HP to end reef location and stops there
      new AutoCoralDriveAndScoreSequence(true, dontScore, end, level, driveTrain, elevator, wrist, coralEffector, algaeGrabber, hopper, rightJoystick, alliance, field),
      
      new DataLogMessage(false, "AutoCoralCycle: End")
    );
  }
}
