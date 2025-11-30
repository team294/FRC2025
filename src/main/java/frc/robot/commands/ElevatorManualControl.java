// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DataLogUtil;

public class ElevatorManualControl extends Command {
  private final Elevator elevator;
  private final CommandXboxController xboxController;

  private boolean rightJoystick;

  // Variables for DataLogging
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogPct =
      new DoubleLogEntry(log, "/ElevatorManualControl/ElevPercent");

  /**
   * Controls the elevator using the Xbox controller joysticks.
   *
   * @param xboxController Xbox controller
   * @param elevator Elevator subsystem
   * @param rightJoystick true = use right joystick, false = use left joystick
   */
  public ElevatorManualControl(
      CommandXboxController xboxController, Elevator elevator, boolean rightJoystick) {
    this.elevator = elevator;
    this.xboxController = xboxController;

    this.rightJoystick = rightJoystick;
    addRequirements(elevator);

    // Prime data logging at boot time
    long timeNow = RobotController.getFPGATime();
    dLogPct.append(-1, timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("ElevatorManualControl: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevPercent = rightJoystick ? -xboxController.getRightY() : -xboxController.getLeftY();
    if ((Math.abs(elevPercent) < OIConstants.joystickDeadband)) elevPercent = 0;

    // Slow the control down
    elevPercent *= ElevatorConstants.maxManualPercentOutput;
    elevator.setElevatorPercentOutput(elevPercent);

    // Log data
    long timeNow = RobotController.getFPGATime();
    dLogPct.append(elevPercent, timeNow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevatorMotors();

    DataLogUtil.writeMessage("ElevatorManualControl: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
