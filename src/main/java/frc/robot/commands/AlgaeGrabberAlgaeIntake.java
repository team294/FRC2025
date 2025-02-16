// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.utilities.FileLog;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeGrabberAlgaeIntake extends Command {
  private final FileLog log;
  private final AlgaeGrabber algaeGrabber;


  /** Intakes algae by running the AlgaeGrabber motor until an algae is detected
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param log LogFile log
   */
  public AlgaeGrabberAlgaeIntake(AlgaeGrabber algaeGrabber, FileLog log) {
    this.algaeGrabber = algaeGrabber;
    this.log = log;

    addRequirements(algaeGrabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Intakes  an algae if one is not already present
    if (!algaeGrabber.isAlgaePresent()) algaeGrabber.setAlgaeGrabberPercentOutput(Constants.AlgaeGrabberConstants.AlgaeGrabberIntakePercent);
    log.writeLog(false, "AlgaeGrabberAlgaeIntake", "Initialize"); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    algaeGrabber.stopAlgaeGrabberMotors(); //If interrupted, stops the algae grabber motor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeGrabber.isAlgaePresent();
  }
}
