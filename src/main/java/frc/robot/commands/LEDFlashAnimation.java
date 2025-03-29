// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;

public class LEDFlashAnimation extends Command {
  private LED led;
  private LEDSegmentRange segment;
  
  /**
   * Creates a new Flash animation that runs until it is interrupted
   * @param led LED subsystem
   * @param segment segment to run animation on
   */
  public LEDFlashAnimation(LED led, LEDSegmentRange segment) {
    this.led = led;
    this.segment = segment;

    addRequirements(led);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
