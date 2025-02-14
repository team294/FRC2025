// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.utilities.FileLog;

public class ElevatorSetPercent extends Command {
  private final FileLog log;
  private final Elevator elevator;
  private final EndEffector endEffector;
  private double percent = 0.0;
  private double timerSeconds = 0.0;
  private final Timer timer;

  /**
   * Sets the percent output to the elevator and ends immediately without turning off.
   * @param percent -1.0 to 1.0 (positive = up, negative = down)
   * @param elevator Elevator subsystem
   * @param endEffector EndEffector subsystem
   * @param log FileLog log
   */
  public ElevatorSetPercent(double percent, Elevator elevator, EndEffector endEffector, FileLog log) {
    this.log = log;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.percent = percent;
    this.timer = new Timer();
    addRequirements(elevator);
  }

  /**
   * Sets the percent output to the elevator and turns off after a number of seconds.
   * @param percent -1.0 to 1.0 (positive = up, negative = down)
   * @param duration in seconds
   * @param elevator Elevator subsystem
   * @param endEffector EndEffector subsystem
   * @param log FileLog log
   */
  public ElevatorSetPercent(double percent, double duration, Elevator elevator, EndEffector endEffector, FileLog log) {
    this.log = log;
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.percent = percent;
    this.timerSeconds = duration;
    this.timer = new Timer();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    if (!endEffector.isPiecePresentInEntry()) elevator.setElevatorPercentOutput(percent);
    log.writeLog(false, "ElevatorSetPercent", "Initialize", "Percent", percent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if (timerSeconds > 0 || interrupted) elevator.stopElevatorMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endEffector.isPiecePresentInEntry() || timerSeconds == 0 || timer.get() >= timerSeconds) return true;
    else return false;
  }
}
