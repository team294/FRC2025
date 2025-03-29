// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.StripEvents;


public class LEDRainbowAnimation extends Command {
  private LED led;
  private LEDSegmentRange segment;

  /**
   * Creates a new CANdle rainbow animation that runs until it is interrupted.
   * @param led LED subsystem
   * @param segment segment to show animation
   */
  public LEDRainbowAnimation(LED led, LEDSegmentRange segment) {
    this.led = led;
    this.segment = segment;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RainbowAnimation anim = new RainbowAnimation(1, .7, segment.count, false, segment.index);
    led.animate(anim);
    led.sendEvent(StripEvents.AUTO_DRIVE_IN_PROGRESS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.clearAnimation();
    led.sendEvent(StripEvents.NEUTRAL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}