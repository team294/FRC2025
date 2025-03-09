// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorWristConstants.ElevatorWristPosition;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristSetAngle extends Command {
  private final Wrist wrist;
  private final FileLog log;
  private double angle;
  private final double tolerance = 3.0; // tolerance of 5 degrees
  private boolean fromShuffleboard;

  /**
   * Sets the target angle for the wrist and moves it to that angle. Ends when the wrist is within 3 degrees 
   * of the target. If the wrist is uncalibrated, this does nothing and ends immediately.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristSetAngle(double angle, Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    this.angle = angle;
    fromShuffleboard = false;
    addRequirements(wrist);
  }

  /**
   * Sets the target angle for the wrist and moves it to that angle. Ends when the wrist is within 3 degrees 
   * of the target. If the wrist is uncalibrated, this does nothing and ends immediately.
   * @param position target WristAngle angle, in degrees (see Constants.WristAngle)
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristSetAngle(ElevatorWristPosition angle, Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    this.angle = angle.wristAngle;
    fromShuffleboard = false;
    addRequirements(wrist);
  }

  /**
   * Sets the target angle for the wrist from Shuffleboard and moves it to that angle. Ends when the wrist is 
   * within 3 degrees of the target. If the wrist is uncalibrated, this does nothing and ends immediately.
   * @param wrist Wrist subsystem
   * @param log FileLog utility
   */
  public WristSetAngle(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    fromShuffleboard = true;

    if (SmartDashboard.getNumber("Wrist Set Angle", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Set Angle", ElevatorWristPosition.CORAL_HP.wristAngle);
    }

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) angle = SmartDashboard.getNumber("Wrist Goal Angle", ElevatorWristPosition.CORAL_HP.wristAngle);
    wrist.setWristAngle(angle);
    log.writeLog(false, "WristSetAngle", "Init", "Target", angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) log.writeLog(false, "WristSetAngle", "Interrupted", "Target", angle, "Current Angle", wrist.getWristAngle());
    else log.writeLog(false, "WristSetAngle", "End", "Target", angle, "Current Angle", wrist.getWristAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !wrist.isWristCalibrated() || Math.abs(wrist.getWristAngle() - wrist.getCurrentWristTarget()) < tolerance;
  }
}
