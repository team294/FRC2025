// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.utilities.FileLog;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HopperSetPercent extends Command {
  private final FileLog log;
  private final Hopper hopper;
  private double hopperPercent = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output to the Hopper and Centering motors from Shuffleboard
   * and ends immediately.
   * @param hopper hopper subsystem
   * @param log
   */
  public HopperSetPercent(Hopper hopper, FileLog log) {
    this.log = log;
    this.hopper = hopper;
    addRequirements(hopper);
    this.fromShuffleboard = true;

    if(SmartDashboard.getNumber("Hopper Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Hopper Percent", 0);
    }
  }

  /**
   * Sets the percent output to the Hopper and Centering motors
   * and ends immediately.
   * @param hopperPercent -1.0 to 1.0 (+ = hopper, - = outtake)
   * @param hopper hopper subsystem
   * @param log
   */  
  public HopperSetPercent(double hopperPercent, Hopper hopper, FileLog log) {
    this.log = log;
    this.hopper = hopper;
    addRequirements(hopper);
    this.fromShuffleboard = false;
    this.hopperPercent = hopperPercent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      hopperPercent = SmartDashboard.getNumber("Hopper Percent", 0.0);
    }
    hopper.setHopperPercentOutput(hopperPercent);

    log.writeLog(false, "HopperSetPercent", "Initialize", "Hopper Percent", hopperPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

