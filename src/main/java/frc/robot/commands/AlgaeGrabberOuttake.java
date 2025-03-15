// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.utilities.FileLog;

public class AlgaeGrabberOuttake extends Command {
  private final AlgaeGrabber algaeGrabber;
  private final FileLog log;
  private final Timer timer;
  private final double seconds;

  /**
   * Outtake algae from the AlgaeGrabber by running the motor until the algae is out of the mechanism.
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param log FileLog utility
   */
  public AlgaeGrabberOuttake(AlgaeGrabber algaeGrabber, FileLog log) {
    this.algaeGrabber = algaeGrabber;
    this.log = log;
    this.timer = new Timer();
    this.seconds = 2.0;
    addRequirements(algaeGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeGrabber.setAlgaeGrabberPercentOutput(AlgaeGrabberConstants.outtakePercent);
    timer.stop();
    timer.reset();

    log.writeLog(false, "AlgaeGrabberOuttake", "Init",
      "Algae Present", algaeGrabber.isAlgaePresent());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeGrabber.stopAlgaeGrabberMotor();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!algaeGrabber.isAlgaePresent()) {
      // If the timer has not been started, start it
      if (!timer.isRunning()) timer.start();
      // Run the motor for slightly longer as a safety measure
      else if (timer.get() >= seconds) return true;     // TODO can we turn off the motor by detecting current?
    }
    return false;
  }
}
