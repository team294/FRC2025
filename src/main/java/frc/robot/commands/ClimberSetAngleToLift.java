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

  /** 
   * Sets Climber Motor percent output until Climber is at the correct angle to climb, then cuts power.
   * Not using position control to avoid robot bouncing during climb.
   * @param climber Climber subsystem
    */
  public ClimberSetAngleToLift(Climber climber) {
    addCommands(
      parallel(
        new ClimberSetPercentOutput(ClimberConstants.maxPercentOutput, climber).until(() -> climber.getClimberAngle() >= ClimberConstants.ClimberAngle.CLIMB_END.value),
        runOnce(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.CLIMBER_LIFTING))
      ).handleInterrupt(() -> LEDEventUtil.sendEvent(LEDEventUtil.StripEvents.NEUTRAL))
    );
  }
}
