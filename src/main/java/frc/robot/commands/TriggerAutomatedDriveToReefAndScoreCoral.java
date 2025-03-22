// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants.ReefLevel;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;

public class TriggerAutomatedDriveToReefAndScoreCoral extends Command {
  JoystickButton joystickButton;
  Trigger xboxButton;

  /**
   * @param xboxController Xbox Controller
   * @param joystickButton Joystick
   * @param level ReefLevel (L1, L2, L3, L4) to score on
   * @param driveTrain DriveTrain subsystem
   * @param elevator Elevator subsystem
   * @param wrist Wrist subsystem
   * @param coralEffector EndEffector subsystem
   * @param algaeGrabber AlgaeGrabber subsystem
   * @param alliance AllianceSelection alliance
   * @param cache TrajectoryCache cache
   * @param field Field field
   * @param log FileLog log
   */
  public TriggerAutomatedDriveToReefAndScoreCoral(Trigger xboxButton, ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
  CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Joystick rightJoystick, AllianceSelection alliance, TrajectoryCache cache, Field field, FileLog log) {

    joystickButton = new JoystickButton(rightJoystick, 1);
    this.xboxButton = xboxButton;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      
    } else {

    }
  }

  @Override
  public boolean isFinished() {
    if (xboxButton.getAsBoolean() && joystickButton.getAsBoolean()) {
      return true;
    } else if (!xboxButton.getAsBoolean() && !joystickButton.getAsBoolean()) {
      end(true);
      return true;
    } else { return false; }
  }
}
