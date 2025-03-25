// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.DataLogUtil;

public class CoralEffectorSetPercent extends Command {
  private final CoralEffector coralEffector;
  
  private double percent = 0.0;
  private boolean fromShuffleboard;

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogPercent = new DoubleLogEntry(log, "/CoralEffectorSetPercent/Percent");

  /**
   * Sets the percent output of the coralEffector from Shuffleboard and ends immediately.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorSetPercent(CoralEffector coralEffector) {
    this.coralEffector = coralEffector;
    
    this.fromShuffleboard = true;
    addRequirements(coralEffector);

    if (SmartDashboard.getNumber("CoralEffector Goal Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("CoralEffector Goal Percent", 0);
    }
  }

  /**
   * Sets the percent output of the coralEffector and ends immediately.
   * @param percent -1.0 to 1.0 (positive = intake/outtake, negative = reverse)
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorSetPercent(double percent, CoralEffector coralEffector) {
    this.coralEffector = coralEffector;
    
    this.percent = percent;
    this.fromShuffleboard = false;
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO check if elevator is in correct position before running motor
    if (fromShuffleboard) percent = SmartDashboard.getNumber("CoralEffector Goal Percent", 0.0);
    coralEffector.setCoralEffectorPercentOutput(percent);

    dLogPercent.append(percent);
    DataLogUtil.writeMessage("CoralEffectorSetPercent: Init, Percent = ", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("CoralEffectorSetPercent: End.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}