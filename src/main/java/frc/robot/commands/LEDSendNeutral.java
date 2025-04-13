// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.BCRColor;
import frc.robot.subsystems.LED;

public class LEDSendNeutral extends InstantCommand {
  private final LED led;

  /**
   * Sends NEUTRAL Strip Event to LEDs 
   * @param led LED subsystem
   */
  public LEDSendNeutral(LED led) {
    this.led = led;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.updateLEDs(BCRColor.NEUTRAL, true);
  }
}
