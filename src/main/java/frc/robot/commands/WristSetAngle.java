// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

public class WristSetAngle extends Command {

  private double angle;
  private final FileLog log;
  private final Wrist wrist;
  private boolean fromShuffleBoard;

  /**
   * Moves wrist to target angle.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   * @param angle target angle in degrees.  (0 = horizontal in front of robot, + = up, - = down)
   */
  public WristSetAngle(double angle, Wrist wrist, FileLog log) {
      this.angle = angle;
      this.wrist = wrist;
      this.log = log;
      fromShuffleBoard = false;

      addRequirements(wrist);
  }

  /**
   * Moves wrist to target position.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   * @param position WristAngle (see Constants)
   */
  public WristSetAngle(WristAngle pos, Wrist wrist, FileLog log) {
    this.angle = pos.value;
    this.wrist = wrist;
    this.log = log;
    fromShuffleBoard = false;

    addRequirements(wrist);
  }

  public WristSetAngle(Wrist wrist, FileLog log) {
    this.wrist = wrist;
    this.log = log;
    fromShuffleBoard = true;

    if(SmartDashboard.getNumber("Wrist Angle to Set", -9999) == -9999) {
      SmartDashboard.putNumber("Wrist Angle to Set", 0);
    }

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleBoard) {
      angle = SmartDashboard.getNumber("Wrist Angle to Set", 0);
    }

    wrist.setWristAngle(angle);
    log.writeLog(false, "WristSetAngle", "Initialize,", "Target", angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "WristSetAngle", "Interrupted", "Target", angle, "Current Angle", wrist.getWristAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !wrist.isEncoderCalibrated() || Math.abs(wrist.getWristAngle() - wrist.getCurrentWristTarget()) < 5.0; // tolerance of 5 degrees 
  }
}
