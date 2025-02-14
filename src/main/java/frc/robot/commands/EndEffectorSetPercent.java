// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;
import frc.robot.utilities.FileLog;

public class EndEffectorSetPercent extends Command {
  private final FileLog log;
  private final EndEffector endEffector;
  private double percent = 0.0;
  private double timerSeconds = 0.0;
  private final Timer timer;

  /**
   * Sets the percent output to the EndEffector and ends immediately without turning off.
   * @param percent -1.0 to 1.0 (positive = outtake, negative = intake)
   * @param endEffector EndEffector subsystem
   * @param log LogFile log
   */
  public EndEffectorSetPercent(double percent, EndEffector endEffector, FileLog log) {
    this.log = log;
    this.endEffector = endEffector;
    this.percent = percent;
    this.timer = new Timer();
    addRequirements(endEffector);
  }

  /**
   * Sets the percent output to the EndEffector and turns off after a number of seconds.
   * @param percent -1.0 to 1.0 (positive = outtake, negative = intake)
   * @param duration number of seconds to run for
   * @param endEffector EndEffector subsystem
   * @param log LogFile log
   */
  public EndEffectorSetPercent(double percent, double duration, EndEffector endEffector, FileLog log) {
    this.log = log;
    this.endEffector = endEffector;
    this.percent = percent;
    this.timerSeconds = duration;
    this.timer = new Timer();
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setEndEffectorPercentOutput(percent);
    log.writeLog(false, "EndEffectorSetPercent", "Initialize", "End Effector Percent", percent);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if (interrupted) endEffector.stopEndEffectorMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timerSeconds == 0 || timer.get() >= timerSeconds) return true;
    else return false;
  }
}