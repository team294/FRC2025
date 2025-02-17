// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.FileLog;

public class HopperSetPercent extends Command {
  private final Hopper hopper;
  private final FileLog log;
  private double hopperPercent = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output of the hopper from Shuffleboard and ends immediately.
   * @param hopper Hopper subsystem
   * @param log FileLog utility
   */
  public HopperSetPercent(Hopper hopper, FileLog log) {
    this.hopper = hopper;
    this.log = log;
    this.fromShuffleboard = true;
    addRequirements(hopper);

    if (SmartDashboard.getNumber("Hopper Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Hopper Percent", 0);
    }
  }

  /**
   * Sets the percent output of the hopper and ends immediately.
   * @param percent -1.0 to 1.0 (positive = intake, negative = reverse)
   * @param hopper Hopper subsystem
   * @param log FileLog utility
   */  
  public HopperSetPercent(double percent, Hopper hopper, FileLog log) {
    this.hopper = hopper;
    this.log = log;
    this.hopperPercent = percent;
    this.fromShuffleboard = false;
    addRequirements(hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) hopperPercent = SmartDashboard.getNumber("Hopper Percent", 0.0);
    hopper.setHopperPercentOutput(hopperPercent);

    log.writeLog(false, "HopperSetPercent", "Init", "Hopper Percent", hopperPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

