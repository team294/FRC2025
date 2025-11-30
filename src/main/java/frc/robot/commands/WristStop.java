// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.DataLogUtil;

public class WristStop extends Command {
  private final Wrist wrist;

  /**
   * Sets the percent output of the wrist to 0 and ends immediately.
   *
   * @param wrist Wrist subsystem
   */
  public WristStop(Wrist wrist) {
    this.wrist = wrist;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.stopWrist();
    DataLogUtil.writeMessage("WristStop: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("WristStop: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
