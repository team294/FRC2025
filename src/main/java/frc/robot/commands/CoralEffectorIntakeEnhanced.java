// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class CoralEffectorIntakeEnhanced extends Command {
  private final CoralEffector coralEffector;
  private final FileLog log;
  
  /**
   * Intakes coral quickly, then ends and auto-holds after the tip of the coral reaches the exit.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorIntakeEnhanced(CoralEffector coralEffector, FileLog log) {
    this.coralEffector = coralEffector;
    this.log = log;
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // If there is no coral present or the coral is not safely in the mechanism, run the motor
    if (!coralEffector.isCoralPresentInExit()) {
      coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.fastIntakePercent); 
    }

    log.writeLog(false, "CoralEffectorIntakeEnhanced", "Init", 
      "Coral in Entry", (coralEffector.isCoralPresentInEntry() ? "TRUE" : "FALSE"),
      "Coral in Exit", (coralEffector.isCoralPresentInExit() ? "TRUE" : "FALSE"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO A different option would be to back off the final position by the (coral velocity)*(time delay @ 20ms)
    if (coralEffector.isCoralPresentInEntry()) {
      coralEffector.setCoralEffectorPosition(coralEffector.getCoralEffectorPosition(), true);
    } else {
      // Back off the position, since due to coral velocity it likely overshot the balanced position between the sensors.
      coralEffector.setCoralEffectorPosition(coralEffector.getCoralEffectorPosition() + CoralEffectorConstants.centerRotationsOvershoot, true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralEffector.isCoralPresentInExit();
  }
}
