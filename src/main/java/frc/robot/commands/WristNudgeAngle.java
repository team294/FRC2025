// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.DataLogUtil;

public class WristNudgeAngle extends InstantCommand {
  private Wrist wrist;
  private DataLogUtil log;
  private double deltaDegrees;
  private boolean fromShuffleboard;

  /**
   * Adjusts the current calibration degrees of the wrist by a small amount.
   * @param deltaDegrees degrees to move (positive = down, negative = up)
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristNudgeAngle(double deltaDegrees, Wrist wrist) {
    this.deltaDegrees = deltaDegrees;
    this.wrist = wrist;
    
    fromShuffleboard = false;
    addRequirements(wrist);
  }

  /**
   * Adjusts the current calibration degrees of the wrist by a small amount from Shuffleboard.
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristNudgeAngle(Wrist wrist) {
    this.wrist = wrist;
    
    fromShuffleboard = true;
    addRequirements(wrist);

    if (SmartDashboard.getNumber("Wrist Nudge Delta Degrees", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Nudge Delta Degrees", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) deltaDegrees = SmartDashboard.getNumber("Wrist Nudge Delta Degrees", 0);
    wrist.nudgeWristAngle(deltaDegrees);
    DataLogUtil.writeLog(false, "WristNudgeAngle", "Init", "Delta", deltaDegrees);
  }
}
