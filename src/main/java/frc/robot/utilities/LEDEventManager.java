package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.commands.LEDAnimationBCR;
import frc.robot.commands.LEDAnimationFlash;
import frc.robot.commands.LEDAnimationRainbow;
import frc.robot.subsystems.LED;


public class LEDEventManager {
    private final LED led;

    private StripEvents previousEventStrip;

    private final LEDAnimationBCR ledAnimationBCR;
    private final LEDAnimationFlash ledAnimationFlashCoral;
    private final LEDAnimationFlash ledAnimationFlashAlgae;
    private final LEDAnimationRainbow ledAnimationRainbow;

    public enum StripEvents {
        MATCH_COUNTDOWN,
        CORAL_MODE,
        CORAL_INTAKING,
        ALGAE_MODE,
        ALGAE_INTAKING,
        AUTO_DRIVE_IN_PROGRESS,
        AUTO_DRIVE_COMPLETE,
        NEUTRAL,
        ROBOT_DISABLED,
        STICKY_FAULT_ACTIVE
      }

      private static final Map<StripEvents, Integer> prioritiesStripEvents = new HashMap<>();
        static {
            prioritiesStripEvents.put(StripEvents.STICKY_FAULT_ACTIVE, 0);
            prioritiesStripEvents.put(StripEvents.CORAL_INTAKING, 1);
            prioritiesStripEvents.put(StripEvents.CORAL_MODE, 2);
            prioritiesStripEvents.put(StripEvents.ALGAE_INTAKING, 3);
            prioritiesStripEvents.put(StripEvents.ALGAE_MODE, 4);
            prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_IN_PROGRESS, 5);
            prioritiesStripEvents.put(StripEvents.AUTO_DRIVE_COMPLETE, 6);
            prioritiesStripEvents.put(StripEvents.NEUTRAL, 7);
            prioritiesStripEvents.put(StripEvents.ROBOT_DISABLED, 8);
        }

    public LEDEventManager(LED led) {
        this.led = led;
        ledAnimationBCR = new LEDAnimationBCR(led, LEDSegmentRange.StripAll);
        ledAnimationFlashCoral = new LEDAnimationFlash(BCRColor.CORAL_MODE, led, LEDSegmentRange.StripAll);
        ledAnimationFlashAlgae = new LEDAnimationFlash(BCRColor.ALGAE_MODE, led, LEDSegmentRange.StripAll);
        ledAnimationRainbow = new LEDAnimationRainbow(led, LEDSegmentRange.StripAll);

        sendEvent(StripEvents.NEUTRAL);
    }

  /**
   * Sends an event to the LED Strips and update the LEDs if necessary.
   * @param event StripEvent event happening
   */
  public void sendEvent(StripEvents event) {
    // Always update state if previous event was neutral or disabled.
    // Do not update if last event was not neutral or disabled and the new event priority is less than the previous.
    // If previous event was algae mode and new event is coral intaking or coral mode, override priorities and update state.
    // If previous event was auto drive complete and new event is coral intaking or algae intaking, override priorities and update state.

    if (
      previousEventStrip != StripEvents.NEUTRAL && previousEventStrip != StripEvents.ROBOT_DISABLED
      && (
        !(previousEventStrip == StripEvents.AUTO_DRIVE_COMPLETE && (event == StripEvents.CORAL_INTAKING || event == StripEvents.ALGAE_INTAKING))
        && !(previousEventStrip == StripEvents.ALGAE_MODE && (event == StripEvents.CORAL_INTAKING || event == StripEvents.CORAL_MODE))
        && getPriority(event) < getPriority(previousEventStrip)
      )
    ) {
      DataLogUtil.writeMessage("LED SendEvent: Return");
      return;
    }

    
    DataLogUtil.writeMessage("LED SendEvent: switch statement");
    switch (event) {
      case CORAL_INTAKING:
        ledAnimationFlashCoral.schedule();
        DataLogUtil.writeMessage("LED Coral Intaking");
        break;
      case CORAL_MODE:
        led.updateLEDs(BCRColor.CORAL_MODE, true);
        DataLogUtil.writeMessage("LED Coral Mode");
        break;
      case ALGAE_INTAKING:
        ledAnimationFlashAlgae.schedule();
        DataLogUtil.writeMessage("LED Algae Intaking");
        break;
      case ALGAE_MODE:
        led.updateLEDs(BCRColor.ALGAE_MODE, true);
        DataLogUtil.writeMessage("LED Algae Mode");
        break;
      case AUTO_DRIVE_IN_PROGRESS:
        ledAnimationRainbow.schedule();
        DataLogUtil.writeMessage("LED Auto Drive in Progress");
        break;
      case AUTO_DRIVE_COMPLETE:
        led.updateLEDs(BCRColor.AUTO_DRIVE_COMPLETE, true);
        DataLogUtil.writeMessage("LED Auto Drive Complete");
        break;
      case ROBOT_DISABLED:
        ledAnimationBCR.schedule();
        DataLogUtil.writeMessage("LED Robot Disabled");
        break;
    //   case STICKY_FAULT_ACTIVE: // TODO top few LEDs on vertical strips turn red, maybe make a boolean so they cant get overriden until its cleared or read the boolean in LED (lastStickyFaultPresentReading)
    //     DataLogUtil.writeMessage("LED Sticky Fault Active");
    //     break;
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
   * Gets the priority level for an event.
   * @param event StripEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private int getPriority(StripEvents event) {
    return event != null ? prioritiesStripEvents.getOrDefault(event, -1) : -1;
  }
}
