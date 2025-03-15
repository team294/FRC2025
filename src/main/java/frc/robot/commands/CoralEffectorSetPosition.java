// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class CoralEffectorSetPosition extends Command {
  private final CoralEffector coralEffector;
  private final FileLog log;
  private final boolean autoHold;
  private double position = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the position of the coralEffector from Shuffleboard and ends when it is within tolerance.
   * @param autoHold true = automatically adjust position so that both coral sensors detect the coral.  
   *   false = hold exact position specified in first parameter.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorSetPosition(boolean autoHold, CoralEffector coralEffector, FileLog log) {
    this.autoHold = autoHold;
    this.coralEffector = coralEffector;
    this.log = log;
    this.fromShuffleboard = true;
    addRequirements(coralEffector);

    if (SmartDashboard.getNumber("CoralEffector Goal Position", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("CoralEffector Goal Position", 0);
    }
  }

  /**
   * Sets the position of the coralEffector and ends when it is within tolerance.
   * @param position position, in motor rotations
   * @param autoHold true = automatically adjust position so that both coral sensors detect the coral.  
   *   false = hold exact position specified in first parameter.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorSetPosition(double position, boolean autoHold, CoralEffector coralEffector, FileLog log) {
    this.coralEffector = coralEffector;
    this.log = log;
    this.position = position;
    this.autoHold = autoHold;
    this.fromShuffleboard = false;
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO check if elevator is in correct position before running motor

    coralEffector.enableFastLogging(true);

    if (fromShuffleboard) position = SmartDashboard.getNumber("CoralEffector Goal Position", 0.0);
    coralEffector.setCoralEffectorPosition(position, autoHold);

    log.writeLog(false, "CoralEffectorSetPosition", "Init", "Set Position", position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.enableFastLogging(false);
    log.writeLog(false, "CoralEffectorSetPosition", "End", "Set Position", position, "Meas Position", coralEffector.getCoralEffectorPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(position - coralEffector.getCoralEffectorPosition()) < CoralEffectorConstants.centeringTolerance;
  }
}