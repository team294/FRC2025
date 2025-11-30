// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ClimberConstants.ServoPosition;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberSetServoPosition extends Command {
  private final Climber climber;
  private double position = 0.0;
  private final boolean fromShuffleboard;
  private final Timer timer;

  /**
   * Sets the climber ratchet servo to a specified position. Waits for the ratchet servo to reach
   * that position before the command ends.
   *
   * @param position 0 -> 1
   * @param climber
   */
  public ClimberSetServoPosition(double position, Climber climber) {
    this.climber = climber;
    this.position = position;
    this.fromShuffleboard = false;
    this.timer = new Timer();

    addRequirements(climber);
  }

  /**
   * Sets the climber ratchet servo to a specified position from Shuffleboard. Waits for the ratchet
   * servo to reach that position before the command ends.
   *
   * @param climber
   */
  public ClimberSetServoPosition(Climber climber) {
    this.climber = climber;
    this.fromShuffleboard = true;
    this.timer = new Timer();

    addRequirements(climber);

    if (SmartDashboard.getNumber("Climb Servo Position Target", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Climb Servo Position Target", 0.0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) position = SmartDashboard.getNumber("Climb Servo Position Target", 0.0);
    position = MathUtil.clamp(position, 0, 1);

    climber.setRatchetPosition(position);

    // Set servo position to UNKNOWN while moving
    climber.setRatchetPositionVariable(ClimberConstants.ServoPosition.UNKNOWN);

    DataLogUtil.writeMessage("ClimberSetServoPosition: Start, target position =", position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      if (position == ServoPosition.DISENGAGED.value)
        climber.setRatchetPositionVariable(ServoPosition.DISENGAGED);
      else if (position == ServoPosition.ENGAGED.value)
        climber.setRatchetPositionVariable(ServoPosition.ENGAGED);
    }
    timer.stop();
    timer.reset();

    DataLogUtil.writeMessage("ClimberSetServoPosition: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= 0.5);
  }
}
