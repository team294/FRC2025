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
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.commands.CANdleBCRAnimation;
import frc.robot.utilities.DataLogUtil;
import frc.robot.utilities.LEDSegment;
import frc.robot.utilities.RobotPreferences;


public class LED extends SubsystemBase {
  private final int logRotationKey;
  private String subsystemName;

  private final CANdle candle;
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  private Timer matchTimer;
  private CANdleEvents previousEventCANdle;
  private StripEvents previousEventStrip;
  private boolean lastStickyFaultPresentReading = false;

  public enum CANdleEvents {
    STICKY_FAULTS_CLEARED,
    STICKY_FAULT_PRESENT,
  }

  Map<CANdleEvents, Integer> prioritiesCANdleEvents = Map.of(
    CANdleEvents.STICKY_FAULTS_CLEARED, 1,
    CANdleEvents.STICKY_FAULT_PRESENT, 1
  );
  
  public enum StripEvents {
    CORAL_MODE,
    ALGAE_MODE,
    AUTO_DRIVE_IN_PROGRESS,
    AUTO_DRIVE_COMPLETE,
    ROBOT_DISABLED,
    SUBSYSTEM_UNCALIBRATED,
    NEUTRAL
  }

  private static final Map<StripEvents, Integer> prioritiesStripEvents = new HashMap<>();
  static {
    prioritiesStripEvents.put(StripEvents.CORAL_MODE, 0);
    prioritiesStripEvents.put(StripEvents.ALGAE_MODE, 1);
    prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_IN_PROGRESS, 2);
    prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_COMPLETE, 3);
    prioritiesStripEvents.put(StripEvents.ROBOT_DISABLED, 4);
    prioritiesStripEvents.put(StripEvents.SUBSYSTEM_UNCALIBRATED, 5);
    prioritiesStripEvents.put(StripEvents.NEUTRAL, 6);
  }

  /**
   * Creates the LED subsystem.
   * @param CANPort the CAN port that the CANdle is on
   * @param subsystemName the name of the subsystem
   * @param matchTimer a timer that tracks the time elapsed in the match
   */
  public LED(int CANPort, String subsystemName, Timer matchTimer) {
    this.subsystemName = subsystemName;
    this.logRotationKey = DataLogUtil.allocateLogRotation();

    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();

    this.matchTimer = matchTimer;

    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.EmptyPatterns.noPatternAnimation));
    }

    sendEvent(StripEvents.NEUTRAL);
  }

  /**
   * Updates the LED strips for a countdown animation.
   * @param percent 0-1 progress through the countdown
   */
  // private void updateLEDsCountdown(double percent) {
  //   double leftCount = LEDSegmentRange.StripLeft.count * percent;
  //   int ledCountLeft = (int) leftCount;

  //   double rightCount = LEDSegmentRange.StripRight.count * percent;
  //   int ledCountRight = (int) rightCount;
    
  //   //TODO change depending on how strips are wired
  //   setLEDs(Color.kRed, LEDSegmentRange.StripLeft.index + LEDSegmentRange.StripLeft.count - ledCountLeft, ledCountLeft); 
  //   setLEDs(Color.kRed, LEDSegmentRange.StripRight.index, ledCountRight);
  // }

  /**
   * Changes the color of the LEDs on either the strips or the CANdle.
   * @param color BCRColor to make LEDs (solid)
   * @param strip true = update strips, false = update CANdle
   */
  private void updateLEDs(BCRColor color, boolean strip) {
    if (strip) {
      setLEDs(color, LEDSegmentRange.StripLeft);
      setLEDs(color, LEDSegmentRange.StripRight);
      setLEDs(color, LEDSegmentRange.StripHorizontal);
    } else {
      setLEDs(color, LEDSegmentRange.CANdle);
    }
  }

  /**
   * Sends an event to the CANdle and update the LEDs if necessary.
   * @param event CANdleEvent event happening
   */
  public void sendEvent(CANdleEvents event) {
    // Do not update if the new event priority is less than the previous
    if (getPriority(event) < getPriority(previousEventCANdle)) return;

    switch (event) {
      case STICKY_FAULT_PRESENT:
        updateLEDs(BCRColor.CANDLE_STICKY_FAULT, false);
        break;
      default:
        updateLEDs(BCRColor.CANDLE_IDLE, false);
        break;
    }

    // Update previous event since we updated the LEDs
    previousEventCANdle = event;
  }

  /**
   * Sends an event to the LED Strips and update the LEDs if necessary.
   * @param event StripEvent event happening
   */
  public void sendEvent(StripEvents event) {
    // If new event is robot disabled, always update LEDs to match it.
    // Always update state if previous event was idle.
    // Do not update if last event was not idle and the new event priority is less than the previous.

    if ((previousEventStrip != StripEvents.NEUTRAL && event != StripEvents.ROBOT_DISABLED) && getPriority(event) < getPriority(previousEventStrip)) return;

    switch (event) {
      // case MATCH_COUNTDOWN:
      //   // Percent of the way through the last 10 seconds of the match (125 seconds in)
      //   double percent = Math.max(matchTimer.get() - 125, 0) / 10.0;
      //   updateLEDsCountdown(percent);
      //   break;
      case SUBSYSTEM_UNCALIBRATED:
        // flash red
        break;
      case AUTO_DRIVE_COMPLETE:
        updateLEDs(BCRColor.AUTO_DRIVE_COMPLETE, true);
        break;
      case AUTO_DRIVE_IN_PROGRESS:
        // rainbow
      case ALGAE_MODE:
        // solid teal when holding, flashing when intaking
      case CORAL_MODE:
        // solid purple when holding only coral, flashing when intaking
      case ROBOT_DISABLED:
        // TODO test CANdleBCRAnimation and figure out how to use it here
        // new CANdleBCRAnimation(this, log);
      default:
        clearAnimation();
        updateLEDs(BCRColor.NEUTRAL, true);
        break;
    }

    // Update previous event since the LEDs were updated
    previousEventStrip = event;
  }

  /**
   * Gets the name of the subsystem.
   * @return the subsystem name
   */
  public String getName() {
    return subsystemName;
  }
  
  /**
   * Clears all animations from the CANdle and LED strips.
   */
  public void clearAnimation() {
    candle.clearAnimation(0);
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      setAnimation(LEDConstants.EmptyPatterns.noPatternAnimation, segmentKey, false);
    }
  }
  
  /**
   * Starts a built-in animation.
   * @param anim animation object to use
   */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Create a pattern based on the segment, leds per gap, and current game object mode
   * @param pattern 1D array of Color objects - each index represents the color of 1 individual LED
   * @param segment LEDSegmentRange constant segment to create pattern for
   * @param ledPerGap # of LEDs 
   * @param algaeMode true = algae mode, false = coral mode
   * @return 1D array of Color objects corresponding to the event
   */
  public Color[] setLEDsScoringPattern(Color[] pattern, LEDSegmentRange segment, int ledPerGap, boolean algaeMode) {
    int count = 0;            // # of LEDs
    int firstGap = -9999;     // LED # for 1st gap
    int secondGap = -9999;    // LED # for 2nd gap
    int thirdGap = -9999;     // LED # for 3rd gap

    switch (segment) {
      case StripLeftSection1:
        count = LEDSegmentRange.StripLeftSection1.count;
        break;
      case StripLeftSection2:
        count = LEDSegmentRange.StripLeftSection2.count;
        // two chunks of LEDs, one gap - 1/2
        firstGap = count / 2;
        break;
      case StripLeftSection3:
        count = LEDSegmentRange.StripLeftSection3.count;
        // three chunks of LEDs, two gaps - 1/3 and 2/3
        firstGap = count / 3;
        secondGap = count * 2 / 3;
        break;
      case StripLeftSection4:
        count = LEDSegmentRange.StripLeftSection4.count;
        // four chunks of LEDs, three gaps - 1/4, 2/4, and 3/4
        firstGap = count / 4;
        secondGap = count / 2;
        thirdGap = count * 3 / 4;
        break;
      default:
        break;
    }

    // set each index of the pattern to the correct color
    for (int i = 0; i < count; i++) {
      // set the first index of the gap to black, as well as additional indices as specified by ledPerGap
      if (i == firstGap || i == secondGap || i == thirdGap) {
        for (int j = 0; j < ledPerGap; j++) {
          pattern[i + j] = new Color(0, 0, 0);
        }
        i += ledPerGap; // increment i by ledPerGap to avoid overriding the gap with a color other than black
      } else {
        // if in algae mode, set to algae color. if in coral mode, set to coral color.
        if (algaeMode) pattern[i] = new Color(BCRColor.ALGAE_MODE.r, BCRColor.ALGAE_MODE.g, BCRColor.ALGAE_MODE.b);
        else pattern[i] = new Color(BCRColor.CORAL_MODE.r, BCRColor.CORAL_MODE.g, BCRColor.CORAL_MODE.b);
      }
    }

    return pattern;
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip.
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
   * Sets the animation for a given led segment.
   * @param animation animation to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[][] animation, LEDSegmentRange segment, boolean loop) {
    segments.get(segment).setAnimation(animation, loop);
  }

   /**
   * Sets the animation for a given LED segment using Color.
   * @param pattern pattern to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[] pattern, LEDSegmentRange segment, boolean loop) {
    Color[][] anim = {pattern};
    segments.get(segment).setAnimation(anim, loop);
  }

  /**
   * Sets LEDs using R, G, and B.
   * @param r red value
   * @param g green value
   * @param b blue value
   */
  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  /**
   * Sets LEDs using Color, an index, and a count.
   * @param color color to set
   * @param index index to start at
   * @param count count of LEDs to set
   */
  public void setLEDs(Color color, int index, int count) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue) * 255, 0, index, count);
  }

  /**
   * Sets LEDs using Color and an index.
   * @param color color to set
   * @param index index to start at
   */
  public void setLEDs(Color color, int index) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, 1);
  }

  /**
   * Sets LEDs using BCRColor and segment values.
   * @param color BCRColor value
   * @param segment segment to light up
   */
  public void setLEDs(BCRColor color, LEDSegmentRange segment) {
    candle.setLEDs(color.r, color.g, color.b, 0, segment.index, segment.count);
  }

  /**
   * Gets the priority level for an event.
   * @param event CANdleEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private int getPriority(CANdleEvents event) {
    return event != null ? prioritiesCANdleEvents.getOrDefault(event, -1) : -1;
  }

  /**
   * Gets the priority level for an event.
   * @param event StripEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private int getPriority(StripEvents event) {
    return event != null ? prioritiesStripEvents.getOrDefault(event, -1) : -1;
  }

  @Override
  public void periodic() {
    if(DataLogUtil.isMyLogRotation(logRotationKey)) {
      // If there is a sticky fault, send sticky fault present event
      if (RobotPreferences.isStickyFaultActive()) {
        sendEvent(CANdleEvents.STICKY_FAULT_PRESENT);
        lastStickyFaultPresentReading = true;
      }

      // If there is not a sticky fault and there previously was, send sticky fault cleared event
      else if (!RobotPreferences.isStickyFaultActive() && lastStickyFaultPresentReading) {
        sendEvent(CANdleEvents.STICKY_FAULTS_CLEARED);
        lastStickyFaultPresentReading = false;
      }

      // If in last 10 seconds of match, send match countdown event
      // if (matchTimer.get() > 125 && matchTimer.get() <= 135) {
      //   sendEvent(StripEvents.MATCH_COUNTDOWN);
      // }
    }
  }
}