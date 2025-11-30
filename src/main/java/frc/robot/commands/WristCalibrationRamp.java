// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.DataLogUtil;

public class WristCalibrationRamp extends Command {
  private final Wrist wrist;

  private double rate; // Ramp rate per execute() cycle = every 20ms
  private double maxPercent;
  private double percent; // Current percent

  /**
   * Slowly ramps voltage to the wrist motors. Used for logging data to calibrate kS and kV for the
   * wrist motors.
   *
   * @param rate rate to increase wrist percent output, in percent (-1 down to +1 up) per second
   * @param maxPercent stop command when wrist percent output (abs value) reaches this value (0 ->
   *     1)
   * @param wrist Wrist subsytsem
   */
  public WristCalibrationRamp(double rate, double maxPercent, Wrist wrist) {
    this.wrist = wrist;

    this.rate = rate * 0.020; // convert to execute() cycles = every 20ms
    this.maxPercent = maxPercent;

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percent = 0.0;
    wrist.enableFastLogging(true);
    DataLogUtil.writeMessage(
        "WristCalibrationRamp: Init, rate =", rate, ", maxPercent =", maxPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Every 20ms, increase percent slightly
    percent += rate;
    wrist.setWristPercentOutput(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopWrist();
    wrist.enableFastLogging(false);
    DataLogUtil.writeMessage("WristCalibrationRamp: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(percent) >= maxPercent
        || (percent > 0
            && wrist.getWristAngle() >= WristConstants.WristAngle.UPPER_LIMIT.value - 10.0)
        || (percent < 0
            && wrist.getWristAngle() <= WristConstants.WristAngle.LOWER_LIMIT.value + 10.0);
  }
}
