// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ServoPosition;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSetRatchet extends Command {
  private final Climber climber;
  private final boolean engaged;
  private final Timer timer;

  private boolean moveNeeded;

  /**
   * Set Climber ratchet engaged / disengaged.  If the the ratchet is already in the requested state,
   * then this command does nothing and ends immediately.  If not, then it sets the ratchet to the given
   * state and waits for it to be in that state before ending.
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
    moveNeeded = (engaged && climber.getRatchetPosition()!=ServoPosition.ENGAGED ||
      !engaged && climber.getRatchetPosition()!=ServoPosition.DISENGAGED);

    if (moveNeeded) {
      climber.setRatchetEngaged(engaged);

      // Set servo position to UNKNOWN while moving
      climber.setRatchetPositionVariable(ClimberConstants.ServoPosition.UNKNOWN);
    }

    timer.start();

    DataLogUtil.writeMessage("ClimberSetRatchet Init, set =", engaged, ", current =", climber.getRatchetPosition(), ", move needed =", moveNeeded);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) climber.setRatchetPositionVariable(engaged ? ClimberConstants.ServoPosition.ENGAGED : ClimberConstants.ServoPosition.DISENGAGED);
    timer.stop();
    timer.reset();

    DataLogUtil.writeMessage("ClimberSetRatchet end, interrupted =", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!moveNeeded || timer.get() >= 0.5);  // TODO calibrate this delay
  }

  public boolean runsWhenDisabled() {
    return true;
  }
}
