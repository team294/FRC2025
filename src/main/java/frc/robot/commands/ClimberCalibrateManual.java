// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberCalibrateManual extends InstantCommand {
  private final Climber climber;
  private DataLogUtil log;
  private double angle;
  private boolean fromShuffleboard;

  /**
   * Manually calibrates the climber, assuming we know its current angle.
   * @param angle the angle the climber is at, in degrees
   * @param climber Climber subsystem
   * @param log FileLog utility
   */
  public ClimberCalibrateManual(double angle, Climber climber) {
    this.climber = climber;
    
    this.angle = angle;
    fromShuffleboard = false;
    addRequirements(climber);
  }

  /**
   * Manually calibrates the climber, assuming we know its current angle.
   * @param climber Climber subsystem
   * @param log FileLog utility
   */
  public ClimberCalibrateManual(Climber climber) {
    this.climber = climber;
    
    fromShuffleboard = true;
    addRequirements(climber);

    if (SmartDashboard.getNumber("Climber Manual Calibration Value", -9999) == -9999) {
      SmartDashboard.putNumber("Climber Manual Calibration Value", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) angle = SmartDashboard.getNumber("Climber Manual Calibration Value", 0);
    climber.calibrateClimberEncoder(angle);

    DataLogUtil.writeLog(true, "ClimberCalibrateManual", "Init", "Angle", angle);
  }
}
