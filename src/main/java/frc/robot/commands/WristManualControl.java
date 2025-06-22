// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.DataLogUtil;

public class WristManualControl extends Command {
  private final Wrist wrist;
  private final CommandXboxController xboxController;
  
  private boolean rightJoystick;

  /**
   * Controls the wrist using the Xbox controller joysticks.
   * @param xboxController Xbox controller
   * @param wrist Wrist subsystem
   * @param rightJoystick true = use right joystick, false = use left joystick
   */
  public WristManualControl(CommandXboxController xboxController, Wrist wrist, boolean rightJoystick) {
    this.wrist = wrist;
    this.xboxController = xboxController;
    
    this.rightJoystick = rightJoystick;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("WristManualControl: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wristPercent = rightJoystick ? -xboxController.getRightY() : -xboxController.getLeftY();
    if ((Math.abs(wristPercent) < OIConstants.joystickDeadband)) wristPercent = 0;

    // Slow the control down
    wristPercent *= WristConstants.maxManualPercentOutput;
    wrist.setWristPercentOutput(wristPercent);

    DataLogUtil.writeMessage("WristManualControl: Execute, Xbox Joystick = ", wristPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopWrist();

    DataLogUtil.writeMessage("WristManualControl: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
