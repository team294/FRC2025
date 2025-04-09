// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.LED;
import frc.robot.utilities.DataLogUtil;;

public class LEDAnimationFlash extends Command {
  private LED led;
  private BCRColor color;
  private int runs;
  
  /**
   * Creates a new flash animation that runs until it is interrupted.
   * @param color BCRColor color to flash
   * @param led LED subsystem
   */
  public LEDAnimationFlash(BCRColor color, LED led) {
    this.led = led;
    this.color = color;

    addRequirements(led);
  }

  @Override
  public void initialize() {
    DataLogUtil.writeMessage("LEDAnimationFlash Init");
    runs = 0;
  }

  @Override
  public void execute() {
    DataLogUtil.writeMessage("LEDAnimationFlash Execute");
    if (runs % 8 == 0) {
      led.updateLEDs(color, true);
    } else if (runs % 17 == 0) {
      led.updateLEDs(BCRColor.NEUTRAL, true);
      runs = 0;
    }

    runs++;
  }

  @Override
  public void end(boolean interrupted) {
    DataLogUtil.writeMessage("LEDAnimationFlash: End");
    led.clearAnimation();
  }

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
