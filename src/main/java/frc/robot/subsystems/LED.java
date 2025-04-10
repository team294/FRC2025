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
  public static BCRColor dashboardColor = BCRColor.NEUTRAL;
  private boolean lastStickyFaultPresentReading = false;

  public enum CANdleEvents {
    STICKY_FAULTS_CLEARED,
    STICKY_FAULT_PRESENT,
}

private static final Map<CANdleEvents, Integer> prioritiesCANdleEvents = new HashMap<>();
    static {
        prioritiesCANdleEvents.put(CANdleEvents.STICKY_FAULTS_CLEARED, 0);
        prioritiesCANdleEvents.put(CANdleEvents.STICKY_FAULT_PRESENT, 0);
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

    candle.configBrightnessScalar(0.25);

    this.matchTimer = matchTimer;
  }

  /**
   * Updates the LED strips for a countdown animation.
   */
  public void updateLEDsCountdown(double percent) {
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
  public void updateLEDs(BCRColor color, boolean strip) {
    if (strip) {
      setLEDs(color, LEDSegmentRange.StripRight);
      setLEDs(color, LEDSegmentRange.StripLeft);
      setLEDs(color, LEDSegmentRange.StripHorizontal);
      // if (lastStickyFaultPresentReading) {
      //   setLEDs(BCRColor.SUBSYSTEM_UNCALIBRATED, LEDSegmentRange.StripLeftTop);
      //   setLEDs(BCRColor.SUBSYSTEM_UNCALIBRATED, LEDSegmentRange.StripRightTop);
      // }
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
        updateLEDsCountdown(Math.max(matchTimer.get() - 125, 0) / 10.0);
      }
    }
  }
}