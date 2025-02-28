package frc.robot.utilities;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LEDSegment {
  private int index, count, frame;
  private boolean loop;
  private Color[][] animation;

  /**
   * Creates an LED segment.
   * @param index start index
   * @param count the number of LEDs from start index
   * @param animation the current animation (animation[frame][LED ID])
   * @param loop true = animation should loop, false = play once then stop
   */
  public LEDSegment(int index, int count, Color[][] animation, boolean loop) {
    this.index = index;
    this.count = count;
    this.animation = animation;
    this.loop = loop;
    this.frame = 0;
  }

  /**
   * Creates an LED segment that loops.
   * @param index start index
   * @param count the number of LEDs from start index
   * @param animation the current animation (animation[frame][LED ID])
   */
  public LEDSegment(int index, int count, Color[][] animation) {
    this(index, count, animation, true);
  }

  /**
   * Sets/resets the animation of the segment, starting it from the beginning.
   * @param animation the animation to play
   * @param loop true = animation should loop, false = play once then stop
   */
  public void setAnimation(Color[][] animation, boolean loop) {
    this.frame = 0;
    this.animation = animation;
    this.loop = loop;
  }

  /**
   * Gets the current frame of the animation, or noPatternStatic if finished.
   * @return the current frame
   */
  public Color[] getCurrentFrame() {
    if (isFinished()) return Constants.LEDConstants.EmptyPatterns.noPatternStatic;
    else return animation[frame];
  }

  /**
   * Gets whether the animation has finished.
   * @return true = animation has finished, false = animation has not finished
   */
  public boolean isFinished() {
    return frame >= animation.length;
  }

  /**
   * Moves the animation to the next frame.
   * @return true = animation has finished in this exact frame (once), false = did not finish on this frame
   */
  public boolean advanceFrame() {
      frame++;
      if (isFinished()) {
          // Reached the end, so either loop or stop
          if (loop) frame = 0;
          else return frame == animation.length;
      }
      return false;
  }
}
