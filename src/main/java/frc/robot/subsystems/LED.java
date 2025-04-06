// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.utilities.*;


public class LED extends SubsystemBase {
  private final int logRotationKey;
  private String subsystemName;

  private final CANdle candle;

  private Timer matchTimer;
  private CANdleEvents previousEventCANdle;
  private StripEvents previousEventStrip;
  private BCRColor dashboardColor = BCRColor.NEUTRAL;
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
    MATCH_COUNTDOWN,
    CORAL_MODE,
    CORAL_INTAKING,
    ALGAE_MODE,
    ALGAE_INTAKING,
    AUTO_DRIVE_IN_PROGRESS,
    AUTO_DRIVE_COMPLETE,
    SUBSYSTEM_UNCALIBRATED,
    SCORING_COMPLETE,
    NEUTRAL,
    ROBOT_DISABLED
  }

  private static final Map<StripEvents, Integer> prioritiesStripEvents = new HashMap<>();
  static {
    prioritiesStripEvents.put(StripEvents.CORAL_INTAKING, 0);
    prioritiesStripEvents.put(StripEvents.CORAL_MODE, 1);
    prioritiesStripEvents.put(StripEvents.ALGAE_INTAKING, 2);
    prioritiesStripEvents.put(StripEvents.ALGAE_MODE, 3);
    // prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_IN_PROGRESS, 4);
    // prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_COMPLETE, 5);
    prioritiesStripEvents.put(StripEvents.SUBSYSTEM_UNCALIBRATED, 6);
    prioritiesStripEvents.put(StripEvents.SCORING_COMPLETE, 7);
    prioritiesStripEvents.put(StripEvents.NEUTRAL, 8);
    prioritiesStripEvents.put(StripEvents.ROBOT_DISABLED, 9);
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
    // this.segments = new HashMap<LEDSegmentRange, LEDSegment>();

    candle.configBrightnessScalar(0.25);

    this.matchTimer = matchTimer;

    sendEvent(StripEvents.NEUTRAL);
  }

  /**
   * Updates the LED strips for a countdown animation.
   * @param percent 0-1 progress through the countdown
   */
  private void updateLEDsCountdown(double percent) {
    double leftCount = LEDSegmentRange.StripRight.count * percent;
    int ledCountLeft = (int) leftCount;

    double rightCount = LEDSegmentRange.StripLeft.count * percent;
    int ledCountRight = (int) rightCount;
    
    setLEDs(Color.kRed, LEDSegmentRange.StripRight.index + LEDSegmentRange.StripRight.count - ledCountLeft, ledCountLeft); 
    setLEDs(Color.kRed, LEDSegmentRange.StripLeft.index, ledCountRight);
  }

  /**
   * Changes the color of the LEDs on either the strips or the CANdle.
   * @param color BCRColor to make LEDs (solid)
   * @param strip true = update strips, false = update CANdle
   */
  private void updateLEDs(BCRColor color, boolean strip) {
    if (strip) {
      setLEDs(color, LEDSegmentRange.StripRight);
      setLEDs(color, LEDSegmentRange.StripLeft);
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
    // Always update state if previous event was neutral or disabled.
    // Do not update if last event was not neutral or disabled and the new event priority is less than the previous.
    // If previous event was algae mode and new event is coral intaking or coral mode, override priorities and update state.

    if (
      previousEventStrip != StripEvents.NEUTRAL && previousEventStrip != StripEvents.ROBOT_DISABLED && previousEventStrip == StripEvents.SCORING_COMPLETE
      && (
        !(previousEventStrip == StripEvents.ALGAE_MODE && (event == StripEvents.CORAL_INTAKING || event == StripEvents.CORAL_MODE))
        || getPriority(event) < getPriority(previousEventStrip)
      )
    ) {
      DataLogUtil.writeMessage("LED SendEvent: Return");
      return;
    }

    
    DataLogUtil.writeMessage("LED SendEvent: switch statement");
    switch (event) {
      case MATCH_COUNTDOWN: // TODO has not been tested
        double percent = Math.max(matchTimer.get() - 125, 0) / 10.0;
        updateLEDsCountdown(percent);
        DataLogUtil.writeMessage("LED Match Countdown");
        break;
      // case AUTO_DRIVE_COMPLETE: // TODO this does not work (has change, needs to be tested)
      //   updateLEDs(BCRColor.AUTO_DRIVE_COMPLETE, true);
      //   dashboardColor = BCRColor.AUTO_DRIVE_COMPLETE;
      //   break;
      case SCORING_COMPLETE:
        updateLEDs(BCRColor.SCORING_COMPLETE, true);
        dashboardColor = BCRColor.SCORING_COMPLETE;
        DataLogUtil.writeMessage("LED Scoring Complete");
        break;
      case ALGAE_MODE:
        updateLEDs(BCRColor.ALGAE_MODE, true);
        dashboardColor = BCRColor.ALGAE_MODE;
        DataLogUtil.writeMessage("LED Algae Mode");
        break;
      case CORAL_MODE:
        updateLEDs(BCRColor.CORAL_MODE, true);
        dashboardColor = BCRColor.CORAL_MODE;
        DataLogUtil.writeMessage("LED Coral Mode");
        break;

      // The events below set the pattern outside of the LED subsystem, see comments

      // LEDAnimationBCR, robot disabled (see RobotContainer.disabledInit())
      case ROBOT_DISABLED:
        dashboardColor = BCRColor.NEUTRAL;
        break;
      // LEDAnimationFlash, Wrist / Elevator uncalibrated (see each subsystem)
      // case SUBSYSTEM_UNCALIBRATED: // TODO has not been tested
      //   dashboardColor = BCRColor.SUBSYSTEM_UNCALIBRATED;
      //   break;
      // LEDAnimationFlash, intaking algae (see AlgaeIntakeSequence)
      case ALGAE_INTAKING:
        dashboardColor = BCRColor.ALGAE_MODE;
        DataLogUtil.writeMessage("LED Algae Intaking");
        break;
      // LEDAnimationFlash, intaking coral (see CoralIntakeSequence)
      case CORAL_INTAKING:
        dashboardColor = BCRColor.CORAL_MODE;
        DataLogUtil.writeMessage("LED Coral Intaking");
        break;
      // // LEDAnimationRainbow, automated drive and score (see AutomatedDriveToReefAndScoreCoral)
      // case AUTO_DRIVE_IN_PROGRESS:
      //   dashboardColor = BCRColor.WHITE;
      //   break;

      default:
        clearAnimation();
        updateLEDs(BCRColor.NEUTRAL, true);
        dashboardColor = BCRColor.NEUTRAL;
        DataLogUtil.writeMessage("LED Neutral");
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
  }
  
  /**
   * Starts a built-in animation.
   * @param anim animation object to use
   */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Sets LEDs using BCRColor constant
   * @param color BCRColor color
   */ 
  public void setLEDs(BCRColor color) {
    candle.setLEDs(color.r, color.g, color.g);
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
   * Sets LEDs using BCRColor, an index, and a count.
   * @param color BCRColor to set
   * @param index index to start at
   * @param count count of LEDs to set
   */
  public void setLEDs(BCRColor color, int index, int count) {
    candle.setLEDs(color.r, color.g, color.b, 0, index, count);
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

      SmartDashboard.putString("LED State", String.format("#%02x%02x%02x", dashboardColor.r, dashboardColor.g, dashboardColor.b));
      SmartDashboard.putNumber("Teleop Timer", matchTimer.get());

      // If in last 10 seconds of match, send match countdown event
      if (matchTimer.get() > 125 && matchTimer.get() <= 135) {
        sendEvent(StripEvents.MATCH_COUNTDOWN);
      }
    }
  }
}