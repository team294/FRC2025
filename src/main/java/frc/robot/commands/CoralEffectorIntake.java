// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.DataLogUtil;

public class CoralEffectorIntake extends Command {
  private final CoralEffector coralEffector;

  /**
   * Intake coral into the coralEffector by running the motor until the coral is safely in the mechanism.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorIntake(CoralEffector coralEffector) {
    this.coralEffector = coralEffector;
    
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // If there is no coral present or the coral is not safely in the mechanism, run the motor
    if (!coralEffector.isCoralPresent() || !coralEffector.isCoralSafelyIn()) {
      coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.intakePercent); 
    }

    DataLogUtil.writeMessage("CoralEffectorIntake: Init, Coral in Entry = ", coralEffector.isCoralPresentInEntry(),
      ", Coral in Exit = ", coralEffector.isCoralPresentInExit());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO check if the coral is safely in before stopping the motor, especially if interrupted
    coralEffector.stopCoralEffectorMotor();
    // if (coralEffector.isCoralPresent()) {
    //   coralEffector.setCoralHoldMode(true);
    // }
    DataLogUtil.writeMessage("CoralEffectorIntake: End.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralEffector.isCoralSafelyIn();
  }
}