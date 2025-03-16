// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristCalibrateManual extends InstantCommand {
  private final Wrist wrist;
  private FileLog log;
  private double angle;
  private boolean fromShuffleboard;

  /**
   * Manually calibrates the wrist, assuming we know its current angle.
   * @param angle the angle the wrist is at, in degrees
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristCalibrateManual(double angle, Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    this.angle = angle;
    fromShuffleboard = false;
    addRequirements(wrist);
  }

  /**
   * Manually calibrates the wrist, assuming we know its current angle.
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristCalibrateManual(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = true;
    addRequirements(wrist);

    if (SmartDashboard.getNumber("Wrist Manual Calibration Value", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Manual Calibration Value", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) angle = SmartDashboard.getNumber("Wrist Manual Calibration Value", 0);
    wrist.calibrateWristEncoder(angle);

    log.writeLog(true, "WristCalibrateManual", "Init", "Angle", angle);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
