// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.LEDEventUtil;

public class ClimberSetAngleToLift extends SequentialCommandGroup {

  /** Sets Climber to Lift Angle
   * @param climber Climber subsystem
    */
  public ClimberSetAngleToLift(Climber climber) {
    addCommands(
      runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL)),
      parallel(
        new ClimberSetAngle(ClimberConstants.ClimberAngle.CLIMB_END, climber),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CLIMBER_LIFTING))
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL))
    );
  }
}
