// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.LED;
import frc.robot.utilities.DataLogUtil;

public class CoralEffectorOuttake extends Command {
  private final CoralEffector coralEffector;
  
  private final Timer timer;
  private final double seconds;

  /**
   * Outtake coral from the coralEffector by running the motor until the coral is out of the mechanism.
   * @param coralEffector CoralEffector subsystem
   */
  public CoralEffectorOuttake(CoralEffector coralEffector) {
    this.coralEffector = coralEffector;
    
    this.timer = new Timer();
    this.seconds = 0.1;
    addRequirements(coralEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralEffector.setCoralEffectorPercentOutput(CoralEffectorConstants.outtakePercent);

    DataLogUtil.writeMessage("CoralEffectorOuttake: Init, Coral in = ", coralEffector.isCoralPresent());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.stopCoralEffectorMotor();
    timer.stop();
    timer.reset();

    DataLogUtil.writeMessage("CoralEffectorOuttake: End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!coralEffector.isCoralPresent()) {
      // If the timer has not been started, start it
      if (!timer.isRunning()) timer.start();
      // Run the motor for slightly longer as a safety measure
      else if (timer.get() >= seconds) {
        DataLogUtil.writeMessage("CoralEffectorOuttake: IsFinished = true - coral is not present and timer has elapsed for long enough.");
        return true;
      }
    }
    return false;
  }
}