// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ClimberSetRatchet extends Command {
  private final Climber climber;
  private final boolean engaged;
  private final Timer timer;

  /**
   * Toggle Climber servo motor engaged / disengaged
   * @param engaged true = engaged, false = disengaged. engaged means we are not to move the climber down.
   * @param climber Climber subsystem
   */
  public ClimberSetRatchet(boolean engaged, Climber climber) {
    this.climber = climber;
    this.engaged = engaged;
    this.timer = new Timer();
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setRatchetEngaged(engaged);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) climber.setServoPositionVariable(ClimberConstants.ServoPosition.UNKNOWN);
    else climber.setServoPositionVariable(engaged ? ClimberConstants.ServoPosition.ENGAGED : ClimberConstants.ServoPosition.DISENGAGED);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= 0.5);
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
