// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  CommandXboxController xboxController;
  JoystickButton joystick;
  // Trigger X;
  // Trigger A;
  // Trigger B;
  // Trigger Y;

  /**
   * @param xboxController Xbox Controller
   * @param joystick Joystick
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
  public TriggerAutomatedDriveToReefAndScoreCoral(CommandXboxController xboxController, ReefLevel level, DriveTrain driveTrain, Elevator elevator, Wrist wrist, 
  CoralEffector coralEffector, AlgaeGrabber algaeGrabber, Joystick rightJoystick, AllianceSelection alliance, TrajectoryCache cache, Field field, FileLog log) {

    this.xboxController = xboxController;
    joystick = new JoystickButton(rightJoystick, 1);

    switch (level) {
      case L1:
        Trigger X = xboxController.x();
      case L2:
        
      case L3:

      case L4:
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
    if (xboxController.x())
    
    return false;
  }
}
