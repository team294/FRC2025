// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.utilities.DataLogUtil;

public class AlgaeGrabberIntake extends Command {
  private final AlgaeGrabber algaeGrabber;

  /**
   * Intake algae into the AlgaeGrabber by running the motor until an algae is detected.
   *
   * @param algaeGrabber AlgaeGrabber subsystem
   */
  public AlgaeGrabberIntake(AlgaeGrabber algaeGrabber) {
    this.algaeGrabber = algaeGrabber;

    addRequirements(algaeGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // If there is no algae present, run the motor
    if (!algaeGrabber.isAlgaePresent())
      algaeGrabber.setAlgaeGrabberPercentOutput(AlgaeGrabberConstants.intakePercent);

    DataLogUtil.writeMessage(
        false, "AlgaeGrabberIntake: Init, Algae Present =", algaeGrabber.isAlgaePresent());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeGrabber.stopAlgaeGrabberMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeGrabber.isAlgaePresent();
  }
}
