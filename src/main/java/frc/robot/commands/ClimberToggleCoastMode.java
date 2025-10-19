// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberToggleCoastMode extends InstantCommand {
  Climber climber;

  /**
   * Toggle Coast Mode on Climber Motor
   * @param climber
   */
  public ClimberToggleCoastMode(Climber climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setCoastMode( !climber.getCoastMode());
    DataLogUtil.writeMessage("ClimberToggleCoastMode: Init");
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
