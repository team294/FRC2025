// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.DataLogUtil;

public class CoralEffectorStop extends Command {
  private final CoralEffector coralEffector;

  /**
   * Sets the percent output of the coralEffector to 0 and ends immediately.
   *
   * @param coralEffector CoralEffector subsystem
   */
  public CoralEffectorStop(CoralEffector coralEffector) {
    this.coralEffector = coralEffector;

    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralEffector.stopCoralEffectorMotor();
    DataLogUtil.writeMessage("CoralEffectorStop: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
