// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;

public class LEDAnimationBCR extends Command {
  private LED led;
  private LEDSegmentRange segment;
  private int cycleCounter, t;
  
  /**
   * Creates a blue and orange (BCR) snaking animation that runs until it is interrupted.
   * @param led LED subsystem
   * @param segment segment to display animation on
   */
  public LEDAnimationBCR(LED led, LEDSegmentRange segment) {
    this.led = led;
    this.segment = segment;

    addRequirements(led);
  }

  @Override
  public void initialize() {
    cycleCounter = 0;
    t = 0;
  }

  @Override
  public void execute() {
    if (t >= 6) {
      // Get the current pattern for the cycle
      BCRColor[] pattern = getPattern(cycleCounter);
  
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

      t = 0;
    } else t++;
  }

  @Override
  public void end(boolean interrupted) {
    led.clearAnimation();
    led.setLEDs(BCRColor.NEUTRAL);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
     * Determines the color pattern based on the cycle counter.
     * @param cycle the current cycle count.
     * @return an array of BCRColors representing the LED pattern.
     */
    private BCRColor[] getPattern(int cycle) {
      switch (cycle) {
        case 0:
          return new BCRColor[]{BCRColor.BLUE, BCRColor.BLUE, BCRColor.ORANGE, BCRColor.ORANGE};
        case 1:
          return new BCRColor[]{BCRColor.ORANGE, BCRColor.BLUE, BCRColor.BLUE, BCRColor.ORANGE};
        case 2:
          return new BCRColor[]{BCRColor.ORANGE, BCRColor.ORANGE, BCRColor.BLUE, BCRColor.BLUE};
        case 3:
          return new BCRColor[]{BCRColor.BLUE, BCRColor.ORANGE, BCRColor.ORANGE, BCRColor.BLUE};
        default:
          return new BCRColor[]{BCRColor.BLUE, BCRColor.BLUE, BCRColor.ORANGE, BCRColor.ORANGE};
      }
    }
}
