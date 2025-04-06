// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;

public class LEDAnimationRainbow extends Command {
  private LED led;
  private LEDSegmentRange segment;

  /**
   * Creates a new rainbow animation that runs until it is interrupted.
   * @param led LED subsystem
   * @param segment segment to show animation
   */
  public LEDAnimationRainbow(LED led, LEDSegmentRange segment) {
    this.led = led;
    this.segment = segment;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RainbowAnimation anim = new RainbowAnimation(0.5, 0.7, segment.count, false, segment.index);
    led.animate(anim);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.clearAnimation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Returns true if the command should run when the robot is disabled.
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}