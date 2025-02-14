// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.LEDSegment;
import frc.robot.utilities.RobotPreferences;


public class LED extends SubsystemBase {
  private final FileLog log;
  private final int logRotationKey;
  private final CANdle candle;
  private String subsystemName;
  // private Timer matchTimer; // implement when robot has LED strips
  private CANdleEvents previousEventCANdle;
  private boolean lastStickyFaultReading;

  public enum CANdleEvents {
    STICKY_FAULT_CLEARED,
    STICKY_FAULT_PRESENT,
    DRIVE_MODE_BRAKE,
    DRIVE_MODE_COAST
  }

  Map<CANdleEvents, Integer> prioritiesCANdleEvents = Map.of(
    CANdleEvents.STICKY_FAULT_CLEARED, 0,
    CANdleEvents.STICKY_FAULT_PRESENT, 1,
    CANdleEvents.DRIVE_MODE_BRAKE, 0,
    CANdleEvents.DRIVE_MODE_COAST, 0
  );

  /**
   * Get the priority level for an event
   * @param event
   * @return priority level as integer - higher value = higher priority
   */
  private int getPriority(CANdleEvents event) {
    return event != null ? prioritiesCANdleEvents.getOrDefault(event, -1) : -1;
  }

  // private Color[] accuracyDisplayPattern = {Color.kRed, Color.kRed};
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  /**
   * Creates the CANdle LED subsystem.
   * @param CANPort
   * @param subsystemName
   * @param matchTimer
   * @param log
   */
  public LED(int CANPort, String subsystemName, Timer matchTimer, FileLog log) {
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    // this.matchTimer = matchTimer; // implement when robot gets LED strips
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    lastStickyFaultReading = RobotPreferences.isStickyFaultActive();

    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;

    // Create the LED segments
    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.Patterns.noPatternAnimation));
    }
  }


  /**
   * Send an event to the CANdle and update LEDs based on event
   * @param event
   */
  public void sendEvent(CANdleEvents event) {
    // allow clearing sticky faults to take priority over sticky fault present
    if (previousEventCANdle == CANdleEvents.STICKY_FAULT_PRESENT && event == CANdleEvents.STICKY_FAULT_CLEARED) {
      previousEventCANdle = null;
    }

    // Do not update if the new event priority is less than the previous event priority
    if (getPriority(event) < getPriority(previousEventCANdle)) return;

    switch (event) {
      case STICKY_FAULT_PRESENT:
        setLEDs(BCRColor.STICKY_FAULT_PRESENT, LEDSegmentRange.CANdle);
        break;
      case DRIVE_MODE_BRAKE:
        setLEDs(BCRColor.DRIVE_MODE_BRAKE, LEDSegmentRange.CANdle);
        break;
      case DRIVE_MODE_COAST:
        setLEDs(BCRColor.DRIVE_MODE_COAST, LEDSegmentRange.CANdle);
        break;
      default:
        setLEDs(BCRColor.CANDLE_IDLE, LEDSegmentRange.CANdle);
        break;
    }
    previousEventCANdle = event;
  }

  /** Get the subsystem's name
   * @return the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }
  
  /** Clear all LEDs */
  public void clearLEDs() {
    setLEDs(0, 0, 0);
  }

  /**
   * Clear the LEDs of a specific segment range
   * @param segment the segment to clear
   */
  public void clearLEDs(LEDSegmentRange segment) {
    setLEDs(0, 0, 0, segment);
  }
  
  /**
   * Clear all animation
   */
  public void clearAnimation() {
    candle.clearAnimation(0);
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      setAnimation(LEDConstants.Patterns.noPatternAnimation, segmentKey, false);
    }
  }
  
  /**
   * Start a built-in animation
   * @param anim
   */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip
   * @param color the color to use
   * @param segment the segment to use
   */
  public void setColor(Color color, LEDSegmentRange segment) {
    Color[] pattern = {color};
    setPattern(pattern, segment);
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip
   * @param pattern the pattern to use
   * @param segment the segment to use
   */
  public void setPattern(Color[] pattern, LEDSegmentRange segment) {
    if (pattern.length == 0) return;
    for (int indexLED = 0, indexPattern = 0; indexLED < segment.count; indexLED++, indexPattern++) {
      if (indexPattern >= pattern.length) indexPattern = 0;
      setLEDs(pattern[indexPattern], segment.index + indexLED);
    }
  }

  /**
   * Sets the animation for a given led segment
   * @param animation animation to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[][] animation, LEDSegmentRange segment, boolean loop) {
    segments.get(segment).setAnimation(animation, loop);
  }

   /**
   * Sets the animation for a given led segment
   * @param pattern pattern to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[] pattern, LEDSegmentRange segment, boolean loop) {
    Color[][] anim = {pattern};
    segments.get(segment).setAnimation(anim, loop);
  }
  
  /**
   * Sets the animation for a given led segment
   * @param color color to display
   * @param segment segment to play animation on
   */
  public void setAnimation(Color color, LEDSegmentRange segment) {
    segments.get(segment).setAnimation(color);
  }
  
  /**
   * Sets the animation for a given led segment
   * @param color BCRColor to display
   * @param segment segment to play animation on
   */
  public void setAnimation(BCRColor color, LEDSegmentRange segment) {
    Color _color = new Color(color.r, color.g, color.b);
    segments.get(segment).setAnimation(_color);
  }

  /**
   * Sets LEDs using only R, G, and B
   * @param r red value
   * @param g green value
   * @param b blue value
   */
  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  /**
   * Takes in color and sets correct RGB values
   * @param color color to set
   */
  public void setLEDs(Color color) {
    setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  public void setLEDs(BCRColor color, LEDSegmentRange segment) {
    setLEDs(color.r, color.g, color.b, segment);
  }

  /**
   * Sets LEDs using RGB, index, and count values
   * @param r red value
   * @param g green value
   * @param b blue value
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(int r, int g, int b, int index, int count) {
    candle.setLEDs(r, g, b, 0, index, count);
  }
  /**
   * Sets LEDs using color and index values
   * @param color color to set
   * @param index index to start at
   */
  public void setLEDs(Color color, int index) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, 1);
  }
  /**
   * Sets LEDs using color, index, and count values
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(Color color, int index, int count) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, count);
  }
  /**
   * Sets LEDs using RGB and segment values
   * @param r red value
   * @param g green value
   * @param b blue value
   * @param segment segment to light up (range)
   */
  public void setLEDs(int r, int g, int b, LEDSegmentRange segment) {
    candle.setLEDs(r, g, b, 0, segment.index, segment.count);
  }
  /**
   * Sets LEDs using BCR color enum (ex: IDLE)
   * @param color color to set
   */
  public void setLEDs(BCRColor color) {
    candle.setLEDs(color.r, color.g, color.b);
  }
  
  /**
   * Sets LEDs using robot state (ex: IDLE)
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(BCRColor color, int index, int count) {
    candle.setLEDs(color.r, color.g, color.b, 0, index, count);
  }

  @Override
  public void periodic() {
    if (log.isMyLogRotation(logRotationKey)) {
      if (RobotPreferences.isStickyFaultActive() && !lastStickyFaultReading) { // set CANdle red if sticky fault present
        sendEvent(CANdleEvents.STICKY_FAULT_PRESENT);
        lastStickyFaultReading = true;
      }
      else if (!RobotPreferences.isStickyFaultActive() && lastStickyFaultReading) { // remove red if sticky fault is no longer present
        sendEvent(CANdleEvents.STICKY_FAULT_CLEARED);
        lastStickyFaultReading = false;
      }
    }
  }
}