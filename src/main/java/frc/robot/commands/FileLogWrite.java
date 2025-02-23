/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.FileLog;

public class FileLogWrite extends Command {
  boolean echo;
  boolean logWhenDisabled;
  String subsystemOrCommand;
  String event;
  FileLog log;
  Object[] paramArray;

  /**
   * Writes to the file log, to be used in command groups.
   * @param echo true = write to the file log and the console, false = write just to the file log
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
   * @param subsystemOrCommand Name of the subsystem or command generating the message
   * @param event Description of the event (ex. start, data, event)
   * @param log FileLog utility
   * @param paramArray List of descriptions and values (variable number of parameters)
   */
  public FileLogWrite(boolean echo, boolean logWhenDisabled, String subsystemOrCommand, String event, FileLog log, Object... paramArray) {
    this.echo = echo;
    this.logWhenDisabled = logWhenDisabled;
    this.subsystemOrCommand = subsystemOrCommand;
    this.event = event;
    this.log = log;
    this.paramArray = paramArray;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (echo) {
      log.writeLogEcho(logWhenDisabled, subsystemOrCommand, event, paramArray);
    } else {
      log.writeLog(logWhenDisabled, subsystemOrCommand, event, paramArray);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}