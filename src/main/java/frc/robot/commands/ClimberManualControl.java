// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.DataLogUtil;

public class ClimberManualControl extends Command {
  private final Climber climber;
  private final CommandXboxController xboxController;
  
  private boolean rightJoystick;

  /**
   * Controls the climber using the Xbox controller joysticks.
   * @param xboxController Xbox controller
   * @param climber Climber subsystem
   * @param rightJoystick true = use right joystick, false = use left joystick
   */
  public ClimberManualControl(CommandXboxController xboxController, Climber climber, boolean rightJoystick) {
    this.climber = climber;
    this.xboxController = xboxController;
    
    this.rightJoystick = rightJoystick;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogUtil.writeMessage("ClimberManualControl: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climberPercent = rightJoystick ? -xboxController.getRightY() : -xboxController.getLeftY();
    if ((Math.abs(climberPercent) < OIConstants.joystickDeadband)) climberPercent = 0;

    // Slow the control down
    climberPercent *= ClimberConstants.maxManualPercentOutput;
    climber.setClimberPercentOutput(climberPercent);

    DataLogUtil.writeMessage("Climber Manual Control: Execute. Percent =", climberPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();

    DataLogUtil.writeMessage("Climber Manual Control: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
