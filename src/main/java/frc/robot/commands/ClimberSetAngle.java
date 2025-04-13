// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberAngle;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSetAngle extends Command {
  private final Climber climber;
  
  private double angle;
  private final double tolerance = 3.0; // tolerance of 3 degrees
  private boolean fromShuffleboard;

  /**
   * Sets the target angle for the climber and moves it to that angle. Ends when the climber is within 3 degrees 
   * of the target. If the climber is uncalibrated, this does nothing and ends immediately.
   * @param angle target angle, in degrees (0 = horizontal in front of robot, positive = up, negative = down)
   * @param climber Climber subsystem
   * @param log FileLog utility
   */
  public ClimberSetAngle(double angle, Climber climber) {
    this.climber = climber;
    
    this.angle = angle;
    fromShuffleboard = false;
    addRequirements(climber);
  }

  /**
   * Sets the target angle for the climber and moves it to that angle. Ends when the climber is within 3 degrees 
   * of the target. If the climber is uncalibrated, this does nothing and ends immediately.
   * @param position target ClimberAngle angle, in degrees (see Constants.ClimberAngle)
   * @param climber Climber subsystem
   * @param log FileLog utility
   */
  public ClimberSetAngle(ClimberAngle pos, Climber climber) {
    this.climber = climber;
    
    this.angle = pos.value;
    fromShuffleboard = false;
    addRequirements(climber);
  }

  /**
   * Sets the target angle for the climber from Shuffleboard and moves it to that angle. Ends when the climber is 
   * within 3 degrees of the target. If the climber is uncalibrated, this does nothing and ends immediately.
   * @param climber Climber subsystem
   */
  public ClimberSetAngle(Climber climber) {
    this.climber = climber;
    
    fromShuffleboard = true;

    if (SmartDashboard.getNumber("Climber Goal Angle", -9999) == -9999) {
      SmartDashboard.putNumber("Climber Goal Angle", climber.getClimberAngle());
    }

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) angle = SmartDashboard.getNumber("Climber Goal Angle", climber.getClimberAngle());
    climber.setClimberAngle(angle);
    DataLogUtil.writeLog(false, "ClimberSetAngle", "Init", "Target", angle);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) DataLogUtil.writeLog(false, "ClimberSetAngle", "Interrupted", "Target", angle, "Current Angle", climber.getClimberAngle());
    else DataLogUtil.writeLog(false, "ClimberSetAngle", "End", "Target", angle, "Current Angle", climber.getClimberAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !climber.isEncoderCalibrated() || Math.abs(climber.getClimberAngle() - climber.getCurrentClimberTarget()) < tolerance;
  }
}
