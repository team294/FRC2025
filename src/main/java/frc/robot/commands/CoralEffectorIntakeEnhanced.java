// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.FileLog;

public class CoralEffectorIntakeEnhanced extends Command {
  private final CoralEffector coralEffector;
  private final Timer timer;
  private final FileLog log;
  private final double seconds;
  private boolean centering;
  
  /**
   * Intakes coral quickly, then slows down the intake motor 0.3 seconds after coral is detected in entry.
   * @param coralEffector CoralEffector subsystem
   * @param log FileLog utility
   */
  public CoralEffectorIntakeEnhanced(CoralEffector coralEffector, FileLog log) {
    this.coralEffector = coralEffector;
    this.timer = new Timer();
    this.seconds = 0.3; // number of seconds before changing from fast intake speed to slow intake speed
    this.log = log;
    this.centering = false;
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralEffector.setCoralHoldMode(false);

    // If there is no coral present or the coral is not safely in the mechanism, run the motor
    if (!coralEffector.isCoralPresent() || !coralEffector.isCoralSafelyIn()) {
      coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.fastIntakePercent); 
    }

    log.writeLog(false, "CoralEffectorIntakeEnhanced", "Init", 
      "Coral in Entry", (coralEffector.isCoralPresentInEntry() ? "TRUE" : "FALSE"),
      "Coral in Exit", (coralEffector.isCoralPresentInExit() ? "TRUE" : "FALSE"));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (centering) {
      if (coralEffector.isCoralPresentInEntry() && !coralEffector.isCoralPresentInExit()) {
        coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.centeringPercent);
      } else if (!coralEffector.isCoralPresentInEntry() && coralEffector.isCoralPresentInExit()) {
        coralEffector.setCoralEffectorPercentOutput(-CoralEffectorConstants.centeringPercent);
      }
    } else {
      // Start the timer when coral is first detected
      if (coralEffector.isCoralPresent() && !timer.isRunning()) {
        timer.start();
      }
      
      // After 0.3 seconds, slow intaking speed
      if (timer.get() >= seconds) {
        coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.slowIntakePercent); 
      }

      // When coral hits the exit, trigger the centering to start
      if (coralEffector.isCoralPresentInExit()) {
        centering = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.stopCoralEffectorMotor();
    coralEffector.setCoralHoldMode(true);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralEffector.isCoralPresentInExit() && coralEffector.isCoralPresentInEntry();
  }
}
