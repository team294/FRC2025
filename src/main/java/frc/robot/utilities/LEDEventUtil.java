package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.commands.LEDAnimationBCR;
import frc.robot.commands.LEDSendNeutral;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;


public class LEDEventUtil {
  private static LED led;
  private static LEDAnimationBCR ledAnimationBCR;
  private static LEDSendNeutral ledSendNeutral;

  private static StripEvents previousEventStrip;

  private static final RainbowAnimation ledAnimationRainbowReef = new RainbowAnimation(1.0, 0.7, LEDSegmentRange.StripAll.count, false, LEDSegmentRange.StripAll.index);
  private static final RainbowAnimation ledAnimationRainbowBarge = new RainbowAnimation(1.0, 0.7, LEDSegmentRange.StripAll.count, true, LEDSegmentRange.StripAll.index);
  private static final StrobeAnimation ledAnimationStrobeAlgae = new StrobeAnimation(BCRColor.ALGAE_MODE.r, BCRColor.ALGAE_MODE.g, BCRColor.ALGAE_MODE.b, 0, 0, LEDSegmentRange.StripAll.count, LEDSegmentRange.StripAll.index);
  private static final StrobeAnimation ledAnimationStrobeCoral = new StrobeAnimation(BCRColor.CORAL_MODE.r, BCRColor.CORAL_MODE.g, BCRColor.CORAL_MODE.b, 0, 0, LEDSegmentRange.StripAll.count, LEDSegmentRange.StripAll.index);

  public enum StripEvents {
    MATCH_COUNTDOWN,
    CORAL_MODE,
    CORAL_INTAKING,
    ALGAE_MODE,
    ALGAE_INTAKING,
    AUTO_DRIVE_IN_PROGRESS_REEF,
    AUTO_DRIVE_IN_PROGRESS_BARGE,
    NEUTRAL,
    ROBOT_DISABLED
  }

  private static final Map<StripEvents, Integer> prioritiesStripEvents = new HashMap<>();
  
  static {
    prioritiesStripEvents.put(StripEvents.CORAL_INTAKING, 1);
    prioritiesStripEvents.put(StripEvents.CORAL_MODE, 2);
    prioritiesStripEvents.put(StripEvents.ALGAE_INTAKING, 3);
    prioritiesStripEvents.put(StripEvents.ALGAE_MODE, 4);
    prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_IN_PROGRESS_REEF, 5);
    prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_IN_PROGRESS_BARGE, 5);
    prioritiesStripEvents.put(StripEvents.NEUTRAL, 6);
    prioritiesStripEvents.put(StripEvents.ROBOT_DISABLED, 7);
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
        !((previousEventStrip == StripEvents.ALGAE_INTAKING || previousEventStrip == StripEvents.ALGAE_MODE) && (event == StripEvents.CORAL_INTAKING || event == StripEvents.CORAL_MODE))
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
      case AUTO_DRIVE_IN_PROGRESS_REEF:
        LED.dashboardColor = BCRColor.NEUTRAL;
        led.animate(ledAnimationRainbowReef);
        DataLogUtil.writeMessage("LED Reef Auto Drive in Progress");
        break;
      case AUTO_DRIVE_IN_PROGRESS_BARGE:
        LED.dashboardColor = BCRColor.NEUTRAL;
        led.animate(ledAnimationRainbowBarge);
        DataLogUtil.writeMessage("LED Barge Auto Drive in Progress");
      case ROBOT_DISABLED:
        LED.dashboardColor = BCRColor.NEUTRAL;
        ledAnimationBCR.schedule();
        DataLogUtil.writeMessage("LED Robot Disabled");
        break;
      default:
        LED.dashboardColor = BCRColor.NEUTRAL;
        led.clearAnimation();
        ledSendNeutral.schedule();
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
    ledSendNeutral = new LEDSendNeutral(led);
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
