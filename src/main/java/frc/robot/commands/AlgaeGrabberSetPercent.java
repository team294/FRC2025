// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.utilities.DataLogUtil;

public class AlgaeGrabberSetPercent extends Command {
  private final AlgaeGrabber algaeGrabber;
  
  private double percent = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output of the algaeGrabber from Shuffleboard and ends immediately.
   * @param algaeGrabber AlgaeGrabber subsystem
   */
  public AlgaeGrabberSetPercent(AlgaeGrabber algaeGrabber) {
    this.algaeGrabber = algaeGrabber;
    
    this.fromShuffleboard = true;
    addRequirements(algaeGrabber);

    if (SmartDashboard.getNumber("AlgaeGrabber Percent", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("AlgaeGrabber Percent", 0);
    }
  }

  /**
   * Sets the percent output of the algaeGrabber and ends immediately.
   * @param percent -1.0 to 1.0 (positive = intake, negative = outtake)
   * @param algaeGrabber AlgaeGrabber subsystem
   */
  public AlgaeGrabberSetPercent(double percent, AlgaeGrabber algaeGrabber) {
    this.algaeGrabber = algaeGrabber;
    
    this.percent = percent;
    this.fromShuffleboard = false;
    addRequirements(algaeGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) percent = SmartDashboard.getNumber("AlgaeGrabber Percent", 0.0);
    algaeGrabber.setAlgaeGrabberPercentOutput(percent);

    DataLogUtil.writeMessage("AlgaeGrabberSetPercent: Init, Percent =", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("AlgaeGrabberSetPercent: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
