// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ServoPosition;
import frc.robot.subsystems.Climber;

public class ClimberSetServoPosition extends Command {
  private final Climber climber;
  private double position = 0.0;
  private final boolean fromShuffleboard;
  private final Timer timer;

  /** Creates a new ClimberSetServoPosition. */
  public ClimberSetServoPosition(double position, Climber climber) {
    this.climber = climber;
    this.position = position;
    this.fromShuffleboard = false;
    this.timer = new Timer();

    addRequirements(climber);
  }

  public ClimberSetServoPosition(Climber climber) {
    this.climber = climber;
    this.fromShuffleboard = true;
    this.timer = new Timer();

    if (SmartDashboard.getNumber("Servo Position Target", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Servo Position Target", 0.0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) position = SmartDashboard.getNumber("Servo Position Target", 0.0);

    climber.setRatchetPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted || position != 1.0 && position != 0.0) climber.setServoPositionVariable(ServoPosition.UNKNOWN);
    else {
      if (position == 1.0) climber.setServoPositionVariable(ServoPosition.DISENGAGED);
      else if (position == 0.0) climber.setServoPositionVariable(ServoPosition.ENGAGED);
      else climber.setServoPositionVariable(ServoPosition.UNKNOWN);
    }
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= 1.0);
  }
}
