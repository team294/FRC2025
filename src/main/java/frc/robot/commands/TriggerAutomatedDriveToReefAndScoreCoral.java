// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants.ReefLevel;

public class TriggerAutomatedDriveToReefAndScoreCoral extends Command {
  CommandXboxController xboxController;
  Joystick rightJoystick;
  Trigger xboxButton;

  /**
   * @param xboxController Xbox Controller
   * @param joystickButton Joystick
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   */
  public TriggerAutomatedDriveToReefAndScoreCoral(CommandXboxController xboxController, Joystick rightJoystick, ReefLevel level) {
    this.rightJoystick = rightJoystick;
    this.xboxController = xboxController;
    
    switch (level) {
      case L1:
        xboxButton = xboxController.x();
        break;
      case L2:
        xboxButton = xboxController.a();
        break;
      case L3:
        xboxButton = xboxController.b();
        break;
      case L4:
        xboxButton = xboxController.y();
        break;
      default:
        xboxButton = xboxController.x();
        break;
    }
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return xboxButton.getAsBoolean() && rightJoystick.getRawButton(1);
    // if both buttons are not pressed, command will be interrupted
  }
}
