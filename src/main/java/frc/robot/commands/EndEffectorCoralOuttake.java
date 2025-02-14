// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.utilities.FileLog;

public class EndEffectorCoralOuttake extends Command {
  private final FileLog log;
  private final EndEffector endEffector;
  private final Timer timer;
  private final double timerSeconds;

  /**
   * Outtake coral by running the endEffector motor until the coral is out of the endEffector.
   * @param endEffector EndEffector subsystem
   * @param log LogFile log
   */
  public EndEffectorCoralOuttake(EndEffector endEffector, FileLog log) {
    this.log = log;
    this.endEffector = endEffector;
    this.timer = new Timer();
    this.timerSeconds = 0.1;
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setEndEffectorPercentOutput(Constants.EndEffectorConstants.endEffectorOuttakePercent);
    log.writeLog(false, "EndEffectorCoralOuttake", "Initialize", 
      "End Effector Piece in Entry", endEffector.isPiecePresentInEntry(), "End Effector Piece in Exit", endEffector.isPiecePresentInExit());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.stopEndEffectorMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!endEffector.isPiecePresent()) {
      if (!timer.isRunning()) timer.start();
      // Run the motor for slightly longer as a safety measure
      else if (timer.get() >= timerSeconds) {
        timer.stop();
        timer.reset();
        return true;
      }
    }
    return false;
  }
}