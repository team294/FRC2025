package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.commands.LEDAnimationBCR;

import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
// import frc.robot.commands.LEDAnimationFlash;
import frc.robot.subsystems.LED;


public class LEDEventUtil {
  private static LED led;
  private static LEDAnimationBCR ledAnimationBCR;

  private static StripEvents previousEventStrip;

  private static final RainbowAnimation ledAnimationRainbow = new RainbowAnimation(0.5, 0.7, LEDSegmentRange.StripAll.count, false, LEDSegmentRange.StripAll.index);
  private static final StrobeAnimation ledAnimationStrobeAlgae = new StrobeAnimation(BCRColor.ALGAE_MODE.r, BCRColor.ALGAE_MODE.g, BCRColor.ALGAE_MODE.b, 0, 0.4, LEDSegmentRange.StripAll.count);
  private static final StrobeAnimation ledAnimationStrobeCoral = new StrobeAnimation(BCRColor.CORAL_MODE.r, BCRColor.CORAL_MODE.g, BCRColor.CORAL_MODE.b, 0, 0.4, LEDSegmentRange.StripAll.count);
  // private final LEDAnimationFlash ledAnimationFlashAlgae;
  // private final LEDAnimationFlash ledAnimationFlashCoral;

  public enum StripEvents {
    MATCH_COUNTDOWN,
    CORAL_MODE,
    CORAL_INTAKING,
    ALGAE_MODE,
    ALGAE_INTAKING,
    AUTO_DRIVE_IN_PROGRESS,
    AUTO_DRIVE_COMPLETE,
    NEUTRAL,
    ROBOT_DISABLED
  }

  private static final Map<StripEvents, Integer> prioritiesStripEvents = new HashMap<>();
  
  static {
    prioritiesStripEvents.put(StripEvents.CORAL_INTAKING, 1);
    prioritiesStripEvents.put(StripEvents.CORAL_MODE, 2);
    prioritiesStripEvents.put(StripEvents.ALGAE_INTAKING, 3);
    prioritiesStripEvents.put(StripEvents.ALGAE_MODE, 4);
    prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_IN_PROGRESS, 5);
    prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_COMPLETE, 6);
    prioritiesStripEvents.put(StripEvents.NEUTRAL, 7);
    prioritiesStripEvents.put(StripEvents.ROBOT_DISABLED, 8);
  }

  /**
   * Sends an event to the LED Strips and update the LEDs if necessary.
   * @param event StripEvent event happening
   */
  public static void sendEvent(StripEvents event) {
    // Always update state if previous event was neutral or disabled.
    // Do not update if last event was not neutral or disabled and the new event priority is less than the previous.
    // If previous event was algae mode and new event is coral intaking or coral mode, override priorities and update state.
    // If previous event was auto drive complete and new event is coral intaking or algae intaking, override priorities and update state.

    if (
      previousEventStrip != StripEvents.NEUTRAL && previousEventStrip != StripEvents.ROBOT_DISABLED
      && (
        !(previousEventStrip == StripEvents.AUTO_DRIVE_COMPLETE && (event == StripEvents.CORAL_INTAKING || event == StripEvents.ALGAE_INTAKING))
        && !((previousEventStrip == StripEvents.ALGAE_INTAKING || previousEventStrip == StripEvents.ALGAE_MODE) && (event == StripEvents.CORAL_INTAKING || event == StripEvents.CORAL_MODE))
        && getPriority(event) < getPriority(previousEventStrip)
      )
    ) {
      DataLogUtil.writeMessage("LED SendEvent: Return");
      return;
    }

    
    DataLogUtil.writeMessage("LED SendEvent: switch statement");
    switch (event) {
      case CORAL_INTAKING:
        LED.dashboardColor = BCRColor.CORAL_MODE;
        led.animate(ledAnimationStrobeCoral);
        DataLogUtil.writeMessage("LED Coral Intaking");
        break;
      case CORAL_MODE:
        LED.dashboardColor = BCRColor.CORAL_MODE;
        led.updateLEDs(BCRColor.CORAL_MODE, true);
        DataLogUtil.writeMessage("LED Coral Mode");
        break;
      case ALGAE_INTAKING:
        LED.dashboardColor = BCRColor.ALGAE_MODE;
        led.animate(ledAnimationStrobeAlgae);
        DataLogUtil.writeMessage("LED Algae Intaking");
        break;
      case ALGAE_MODE:
        LED.dashboardColor = BCRColor.ALGAE_MODE;
        led.updateLEDs(BCRColor.ALGAE_MODE, true);
        DataLogUtil.writeMessage("LED Algae Mode");
        break;
      case AUTO_DRIVE_IN_PROGRESS:
        LED.dashboardColor = BCRColor.WHITE;
        led.animate(ledAnimationRainbow);
        DataLogUtil.writeMessage("LED Auto Drive in Progress");
        break;
      case AUTO_DRIVE_COMPLETE:
        LED.dashboardColor = BCRColor.AUTO_DRIVE_COMPLETE;
        led.updateLEDs(BCRColor.AUTO_DRIVE_COMPLETE, true);
        DataLogUtil.writeMessage("LED Auto Drive Complete");
        break;
      case ROBOT_DISABLED:
        LED.dashboardColor = BCRColor.WHITE;
        ledAnimationBCR.schedule();
        DataLogUtil.writeMessage("LED Robot Disabled");
        break;
      default:
        led.clearAnimation();
        led.updateLEDs(BCRColor.NEUTRAL, true);
        DataLogUtil.writeMessage("LED Neutral");
        break;
    }

    // Update previous event since the LEDs were updated
    previousEventStrip = event;
  }

  /**
   * This method must be called once in RobotContainer
   * @param ledInstance LED subsystem
   */
  public static void start(LED ledInstance) {
    led = ledInstance;
    ledAnimationBCR = new LEDAnimationBCR(led, LEDSegmentRange.StripAll);
  }

  /**
   * Gets the priority level for an event.
   * @param event StripEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private static int getPriority(StripEvents event) {
    return event != null ? prioritiesStripEvents.getOrDefault(event, -1) : -1;
  }
}
