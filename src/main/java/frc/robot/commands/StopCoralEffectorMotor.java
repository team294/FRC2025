// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class StopCoralEffectorMotor extends Command {
  private final CoralEffector coralEffector;
  private final FileLog log;

  /**
   * Sets the percent output of the coralEffector to 0 and ends immediately.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public StopCoralEffectorMotor(CoralEffector coralEffector, FileLog log) {
    this.coralEffector = coralEffector;
    this.log = log;
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralEffector.stopCoralEffectorMotor();
    log.writeLog(false, "StopCoralEffectorMotor", "Init");
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
