// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberCalibrationRamp extends Command {
  private final Climber climber;
  

  private double rate;        // Ramp rate per execute() cycle = every 20ms
  private double maxPercent;
  private double percent;     // Current percent

  /**
   * Slowly ramps voltage to the climber motors. Used for logging data to calibrate
   * kS and kV for the climber motors.
   * @param rate rate to increase climber percent output, in percent (-1 down to +1 up) per second
   * @param maxPercent stop command when climber percent output (abs value) reaches this value (0 -> 1)
   * @param climber Climber subsystem
   */
  public ClimberCalibrationRamp(double rate, double maxPercent, Climber climber) {
    this.climber = climber;
    
    this.rate = rate * 0.020; // convert to execute() cycles = every 20ms
    this.maxPercent = maxPercent;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percent = 0.0;
    climber.enableFastLogging(true);
    DataLogUtil.writeMessage("ClimberCalibrationRamp: Init, rate = ", rate, ", max percent = ", maxPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Every 20ms, increase percent slightly
    percent += rate;
    climber.setClimberPercentOutput(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
    climber.enableFastLogging(false);
    DataLogUtil.writeMessage("ClimberCalibrationRamp: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(percent) >= maxPercent;
  }
}
