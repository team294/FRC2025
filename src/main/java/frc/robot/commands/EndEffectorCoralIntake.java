// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import frc.robot.utilities.FileLog;

public class EndEffectorCoralIntake extends Command {
  private final FileLog log;
  private final EndEffector endEffector;

  /**
   * Intake coral by running the endEffector motor until the coral is safely in the endEffector.
   * @param endEffector EndEffector subsystem
   * @param log LogFile log
   */
  public EndEffectorCoralIntake(EndEffector endEffector, FileLog log) {
    this.endEffector = endEffector;
    this.log = log;
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (endEffector.isPiecePresentInEntry() || !endEffector.isPiecePresent()) endEffector.setEndEffectorPercentOutput(Constants.EndEffectorConstants.endEffectorIntakePercent);
    log.writeLog(false, "EndEffectorCoralIntake", "Initialize", 
      "End Effector Piece in Entry", (endEffector.isPiecePresentInEntry() ? "TRUE" : "FALSE"), "End Effector Piece in Exit", (endEffector.isPiecePresentInExit() ? "TRUE" : "FALSE"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If command is interrupted, check if the piece is safely in before stopping motor
    // if (!interrupted || endEffector.isPieceSafelyIn()) endEffector.stopEndEffectorMotor();
    endEffector.stopEndEffectorMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endEffector.isPieceSafelyIn()) return true;
    return false;
  }
}