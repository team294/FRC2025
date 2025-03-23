// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.DataLogUtil;

public class WristSetPercent extends Command {
  private final Wrist wrist;
  private final DataLogUtil log;
  private double percent = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output of the coralEffector from Shuffleboard.
   * <b>NOTE: This command does not end. When interrupted, it turns off the wrist motor.
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristSetPercent(Wrist wrist, DataLogUtil log) {
    this.wrist = wrist;
    this.log = log;
    this.fromShuffleboard = true;
    addRequirements(wrist);

    if (SmartDashboard.getNumber("Wrist Goal Percent", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Goal Percent", 0);
    }
  }

  /**
   * Sets the percent output of the wrist.
   * <b>NOTE: This command does not end. When interrupted, it turns off the wrist motor.
   * @param percent -1.0 to 1.0 (positive = up, negative = down)
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristSetPercent(double percent, Wrist wrist, DataLogUtil log) {
    this.log = log;
    this.wrist = wrist;
    this.percent = percent;
    this.fromShuffleboard = false;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) percent = SmartDashboard.getNumber("Wrist Goal Percent", 0);
    wrist.setWristPercentOutput(percent);
    log.writeLog(false, "WristSetPercent", "Init", "Percent", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
