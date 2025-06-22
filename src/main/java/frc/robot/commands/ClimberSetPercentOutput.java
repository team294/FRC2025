// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ServoPosition;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSetPercentOutput extends Command {
  private final Climber climber;
  
  private double percent = 0.0;
  private boolean fromShuffleboard;

  /**
   * Sets the percent output of the climber from Shuffleboard.
   * <b>NOTE: This command does not end. When interrupted, it turns off the climber motor.
   * @param climber Climber subsystem
   */
  public ClimberSetPercentOutput(Climber climber) {
    this.climber = climber;
    
    this.fromShuffleboard = true;
    addRequirements(climber);

    if (SmartDashboard.getNumber("Climber Goal Percent", -9999) == -9999) {
      SmartDashboard.putNumber("Climber Goal Percent", 0);
    }
  }

  /**
   * Sets the percent output of the climber.
   * <b>NOTE: This command does not end. When interrupted, it turns off the climber motor.
   * @param percent -1.0 to 1.0 (positive = up, negative = down)
   * @param climber Climber subsystem
   */
  public ClimberSetPercentOutput(double percent, Climber climber) {
    
    this.climber = climber;
    this.percent = percent;
    this.fromShuffleboard = false;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) percent = SmartDashboard.getNumber("Climber Goal Percent", 0);
    if (climber.getRatchetPosition() == ServoPosition.DISENGAGED || percent >= 0) climber.setClimberPercentOutput(percent);
    DataLogUtil.writeMessage("ClimberSetPercentOutput: Init, Percent = ", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // will be interrupted in the climbing lift sequence
  }
}
