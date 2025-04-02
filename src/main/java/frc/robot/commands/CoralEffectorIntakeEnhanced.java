// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.DataLogUtil;

public class CoralEffectorIntakeEnhanced extends Command {
  private final CoralEffector coralEffector;
  private boolean startedWithCoral;
  
  /**
   * Intakes coral quickly, then ends and auto-holds after the tip of the coral reaches the exit.
   * @param coralEffector CoralEffector subsystem
   */
  public CoralEffectorIntakeEnhanced(CoralEffector coralEffector) {
    this.coralEffector = coralEffector;
    
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // If there is no coral present or the coral is not safely in the mechanism, run the motor
    startedWithCoral = coralEffector.isCoralPresent();
    if (!startedWithCoral) {
      coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.fastIntakePercent); 
    }

    DataLogUtil.writeMessage("CoralEffectorIntakeEnhanced: Init, Coral in =", coralEffector.isCoralPresent());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Back off the position, since due to coral velocity it likely overshot the balanced position between the sensors.
    // A different option would be to add to the final position by the (coral velocity)*(time delay @ 20ms).
    if (!startedWithCoral) {
      coralEffector.setCoralEffectorPosition(coralEffector.getCoralEffectorPosition() + CoralEffectorConstants.centerRotationsUndershoot, true);
    }

    DataLogUtil.writeMessage("CoralEffectorIntakeEnhanced: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralEffector.isCoralPresent();
  }
}
