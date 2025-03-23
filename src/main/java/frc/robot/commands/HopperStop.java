// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.DataLogUtil;

public class HopperStop extends Command {
  private final Hopper hopper;
  private final DataLogUtil log;
 
  /**
   * Sets the percent output of the hopper to 0 and ends immediately.
   * @param hopper Hopper subsystem
   * @param log FileLog utility
   */
  public HopperStop(Hopper hopper, DataLogUtil log) {
    this.hopper = hopper;
    this.log = log;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.setHopperPercentOutput(0);
    log.writeLog(false, "HopperStop", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
