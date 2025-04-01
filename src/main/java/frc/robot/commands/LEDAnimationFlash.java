// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.StripEvents;;

public class LEDAnimationFlash extends Command {
  private LED led;
  private LEDSegmentRange segment;
  private StripEvents event;
  private BCRColor color;
  private int runs;
  
  /**
   * Creates a new flash animation that runs until it is interrupted.
   * @param led LED subsystem
   * @param segment segment to run animation on
   */
  public LEDAnimationFlash(StripEvents event, LED led, LEDSegmentRange segment) {
    this.led = led;
    this.segment = segment;
    this.event = event;

    addRequirements(led);
  }

  @Override
  public void initialize() {
    runs = 0;
    led.sendEvent(event);

    switch (event) {
      case SUBSYSTEM_UNCALIBRATED:
        color = BCRColor.SUBSYSTEM_UNCALIBRATED;
        break;
      case ALGAE_INTAKING:
        color = BCRColor.ALGAE_MODE;
        break;
      case CORAL_INTAKING:
        color = BCRColor.CORAL_MODE;
        break;
      default:
        break;
    }
  }

  @Override
  public void execute() {
    if (runs % 16 == 0) {
      led.setLEDs(color, segment);
    } else if (runs % 33 == 0) {
      led.setLEDs(BCRColor.NEUTRAL, segment);
      runs = 0;
    }

    runs++;
  }

  @Override
  public void end(boolean interrupted) {
    led.sendEvent(StripEvents.NEUTRAL);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
