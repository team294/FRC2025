package frc.robot.utilities;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants.LEDSegments;
import frc.robot.commands.LEDAnimationBCR;
import frc.robot.commands.LEDSendNeutral;
import frc.robot.subsystems.LED;
import java.util.HashMap;
import java.util.Map;

public class LEDEventUtil {
  private static LED led;
  private static LEDAnimationBCR ledAnimationBCR;
  private static LEDSendNeutral ledSendNeutral;

  private static StripEvents previousEventStrip;

  private static final RainbowAnimation ledAnimationRainbowReef =
      new RainbowAnimation(1.0, 0.8, LEDSegments.StripAll.count, false, LEDSegments.StripAll.index);
  private static final RainbowAnimation ledAnimationRainbowBarge =
      new RainbowAnimation(1.0, 0.8, LEDSegments.StripAll.count, true, LEDSegments.StripAll.index);
  private static final StrobeAnimation ledAnimationStrobeAlgae =
      new StrobeAnimation(
          BCRColor.ALGAE_MODE.r,
          BCRColor.ALGAE_MODE.g,
          BCRColor.ALGAE_MODE.b,
          0,
          0,
          LEDSegments.StripAll.count,
          LEDSegments.StripAll.index);
  private static final StrobeAnimation ledAnimationStrobeCoral =
      new StrobeAnimation(
          BCRColor.CORAL_MODE.r,
          BCRColor.CORAL_MODE.g,
          BCRColor.CORAL_MODE.b,
          0,
          0,
          LEDSegments.StripAll.count,
          LEDSegments.StripAll.index);
  private static final StrobeAnimation ledAnimationStrobeClimber =
      new StrobeAnimation(
          BCRColor.BLUE.r,
          BCRColor.BLUE.g,
          BCRColor.BLUE.b,
          0,
          0,
          LEDSegments.StripAll.count,
          LEDSegments.StripAll.index);

  public enum StripEvents {
    MATCH_COUNTDOWN,
    CORAL_MODE,
    CORAL_INTAKING,
    ALGAE_MODE,
    ALGAE_INTAKING,
    AUTOMATED_DRIVING_REEF,
    AUTOMATED_DRIVING_BARGE,
    CLIMBER_PREPPING,
    CLIMBER_PREPPED,
    CLIMBER_LIFTING,
    NEUTRAL,
    ROBOT_DISABLED
  }

  private static final Map<StripEvents, Integer> prioritiesStripEvents = new HashMap<>();

  static {
    prioritiesStripEvents.put(StripEvents.CORAL_INTAKING, 1);
    prioritiesStripEvents.put(StripEvents.CORAL_MODE, 2);
    prioritiesStripEvents.put(StripEvents.ALGAE_INTAKING, 3);
    prioritiesStripEvents.put(StripEvents.ALGAE_MODE, 4);
    prioritiesStripEvents.put(StripEvents.AUTOMATED_DRIVING_REEF, 5);
    prioritiesStripEvents.put(StripEvents.AUTOMATED_DRIVING_BARGE, 5);
    prioritiesStripEvents.put(StripEvents.CLIMBER_PREPPING, 6);
    prioritiesStripEvents.put(StripEvents.CLIMBER_PREPPED, 7);
    prioritiesStripEvents.put(StripEvents.CLIMBER_LIFTING, 8);
    prioritiesStripEvents.put(StripEvents.NEUTRAL, 9);
    prioritiesStripEvents.put(StripEvents.ROBOT_DISABLED, 10);
  }

  /**
   * Sends an event to the LED Strips and update the LEDs if necessary.
   *
   * @param event StripEvent event happening
   */
  public static void sendEvent(StripEvents event) {
    // Always update state if previous event was neutral or disabled.
    // Do not update if last event was not neutral or disabled and the new event priority is less
    // than the previous.
    // If previous event was algae mode and new event is coral intaking or coral mode, override
    // priorities and update state.
    // If previous event was automated drive (either type) and new event is coral or algae mode,
    // override priorities and update state.

    if (previousEventStrip != StripEvents.NEUTRAL
        && previousEventStrip != StripEvents.ROBOT_DISABLED
        && (!((previousEventStrip == StripEvents.ALGAE_INTAKING
                    || previousEventStrip == StripEvents.ALGAE_MODE)
                && (event == StripEvents.CORAL_INTAKING || event == StripEvents.CORAL_MODE))
            && !((previousEventStrip == StripEvents.AUTOMATED_DRIVING_REEF)
                || (previousEventStrip == StripEvents.AUTOMATED_DRIVING_BARGE)
                    && (event == StripEvents.CORAL_MODE || event == StripEvents.ALGAE_MODE))
            && getPriority(event) < getPriority(previousEventStrip))) {
      DataLogUtil.writeMessage("LED Strips SendEvent: Return");
      return;
    }

    DataLogUtil.writeMessage("LED Strips SendEvent: switch statement");
    switch (event) {
      case CORAL_INTAKING:
        LED.dashboardColor = BCRColor.CORAL_MODE;
        led.clearAnimation();
        led.animate(ledAnimationStrobeCoral);
        DataLogUtil.writeMessage("LED Strips Coral Intaking");
        break;
      case CORAL_MODE:
        LED.dashboardColor = BCRColor.CORAL_MODE;
        led.clearAnimation();
        led.updateLEDs(BCRColor.CORAL_MODE, true);
        DataLogUtil.writeMessage("LED Strips Coral Mode");
        break;
      case ALGAE_INTAKING:
        LED.dashboardColor = BCRColor.ALGAE_MODE;
        led.clearAnimation();
        led.animate(ledAnimationStrobeAlgae);
        DataLogUtil.writeMessage("LED Strips Algae Intaking");
        break;
      case ALGAE_MODE:
        LED.dashboardColor = BCRColor.ALGAE_MODE;
        led.clearAnimation();
        led.updateLEDs(BCRColor.ALGAE_MODE, true);
        DataLogUtil.writeMessage("LED Strips Algae Mode");
        break;
      case AUTOMATED_DRIVING_REEF:
        LED.dashboardColor = BCRColor.ORANGE;
        led.clearAnimation();
        led.animate(ledAnimationRainbowReef);
        DataLogUtil.writeMessage("LED Strips Reef Auto Drive in Progress");
        break;
      case AUTOMATED_DRIVING_BARGE:
        LED.dashboardColor = BCRColor.ORANGE;
        led.clearAnimation();
        led.animate(ledAnimationRainbowBarge);
        DataLogUtil.writeMessage("LED Strips Barge Auto Drive in Progress");
        break;
      case CLIMBER_PREPPING:
        LED.dashboardColor = BCRColor.BLUE;
        led.clearAnimation();
        led.animate(ledAnimationStrobeClimber);
        DataLogUtil.writeMessage("LED Strips Climber Prepping");
        break;
      case CLIMBER_PREPPED:
        LED.dashboardColor = BCRColor.BLUE;
        led.clearAnimation();
        led.updateLEDs(BCRColor.BLUE, true);
        DataLogUtil.writeMessage("LED Strips Climber Prepped");
        break;
      case CLIMBER_LIFTING:
        LED.dashboardColor = BCRColor.BLUE;
        led.clearAnimation();
        led.animate(ledAnimationRainbowBarge);
        DataLogUtil.writeMessage("LED Strips Climber Lifting");
        break;
      case ROBOT_DISABLED:
        LED.dashboardColor = BCRColor.NEUTRAL;
        led.clearAnimation();
        ledAnimationBCR.schedule();
        DataLogUtil.writeMessage("LED Strips Robot Disabled");
        break;
      default:
        LED.dashboardColor = BCRColor.NEUTRAL;
        led.clearAnimation();
        ledSendNeutral.schedule();
        DataLogUtil.writeMessage("LED Strips Neutral");
        break;
    }

    // Update previous event since the LEDs were updated
    previousEventStrip = event;
  }

  /**
   * This method must be called once in RobotContainer
   *
   * @param ledInstance LED subsystem
   */
  public static void start(LED ledInstance) {
    led = ledInstance;
    ledAnimationBCR = new LEDAnimationBCR(led, LEDSegments.StripAll);
    ledSendNeutral = new LEDSendNeutral(led);
  }

  /**
   * Gets the priority level for an event.
   *
   * @param event StripEvents event
   * @return priority level integer (higher value = higher priority), default is -1
   */
  private static int getPriority(StripEvents event) {
    return event != null ? prioritiesStripEvents.getOrDefault(event, -1) : -1;
  }
}
