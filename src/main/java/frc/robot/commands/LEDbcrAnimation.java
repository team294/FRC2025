// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.StripEvents;

public class LEDbcrAnimation extends Command {
  private LED led;
  private LEDSegmentRange segment;
  private int cycleCounter = 0;
  
  /**
   * Creates a blue and orange snaking animation in groups of 2
   * @param led LED subsystem
   * @param segment segment to display animation on
   */
  public LEDbcrAnimation(LED led, LEDSegmentRange segment) {
    this.led = led;
    this.segment = segment;

    addRequirements(led);
  }

  @Override
  public void initialize() {
    led.sendEvent(StripEvents.ROBOT_DISABLED);
  }

  @Override
  public void execute() {
    // Get the current pattern for the cycle
    Color[] pattern = getPattern(cycleCounter);

    // Get the index and count from the segment
    int segmentIndex = segment.index;
    int segmentCount = segment.count;


    // Loop through the LEDs in the segment and apply the pattern
    for (int i = 0; i < segmentCount; i += 2) {
      int colorIndex = (i / 2) % 4; // Determine which color pair to use based on the cycle
      led.setLEDs(pattern[colorIndex], segmentIndex + i, 2);
    }

    // Increment the cycleCounter and reset it after a full cycle
    cycleCounter = (cycleCounter + 1) % 4;
  }

  @Override
  public void end(boolean interrupted) {
    led.clearAnimation();
    led.sendEvent(StripEvents.NEUTRAL);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
     * Determines the color pattern based on the cycle counter.
     * @param cycle the current cycle count.
     * @return an array of Colors representing the LED pattern.
     */
    private Color[] getPattern(int cycle) {
      switch (cycle) {
        case 0:
          return new Color[]{Color.kBlue, Color.kBlue, Color.kOrange, Color.kOrange};
        case 1:
          return new Color[]{Color.kOrange, Color.kBlue, Color.kBlue, Color.kOrange};
        case 2:
          return new Color[]{Color.kOrange, Color.kOrange, Color.kBlue, Color.kBlue};
        case 3:
          return new Color[]{Color.kBlue, Color.kOrange, Color.kOrange, Color.kBlue};
        default:
          return new Color[]{Color.kBlue, Color.kBlue, Color.kOrange, Color.kOrange};
      }
    }
}
