package frc.robot.utilities;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileGenerator {
  Elevator elevator;
  FileLog log;

  private boolean profileEnabled = false;

  private double currentMPDistance;   // Distance that should have been traveled in the current motion profile (always positive)
  private double targetMPDistance;    // Total distance to be traveled in the current motion profile (always positive)

  private double initialPosition;     // Initial position from the bottom of the elevator, in inches
  private double finalPosition;       // Final position from the bottom of the elevator, in inches

  private double maxVelocity = 45.0;  // Max velocity, in inches/second TODO CALIBRATE FOR 2025
  private double currentMPVelocity;   // Velocity that it should be at in the current motion profile

  private double maxAcceleration = 128.0;                       // Max acceleration, in inches/second^2 TODO CALIBRATE FOR 2025
  private double stoppingAcceleration = .75 * maxAcceleration;  // Limit the stopping acceleration to 75% of the max acceleration
  private double currentMPAcceleration;                         // Acceleration that it should be at in the current motion profile
  private boolean approachingTarget = false;                    // true = close enough to target to be decelerating, false = not close enough

  private double dt;  // delta T (time)

  private double directionSign; // +1 if finalPosition > initialPosition, -1 if not

  private long startTime, lastTime;

  private double prevError, error, intError;

  double percentPowerFF = 0;
  double percentPowerFB = 0;

  // CALIBRATED
  private double kFF = 0.30 / ElevatorConstants.compensationVoltage;     // In pct-output.  Was 0.35
  private double kSu = 0.10 / ElevatorConstants.compensationVoltage;     // In pct-output
  private double kVu = 0.126 / ElevatorConstants.compensationVoltage;     // In (pct-output)/(in/s)
  private double kAu = 0.000;      // In (pct-output)/(in/s^2)     2023 = 0.0006
  private double kPu = 0.1;        // In (pct-output)/(in)
  private double kIu = 0;    
  private double kDu = 0.0;      // 2023 = 0.02
  private double kSd = 0.10 / ElevatorConstants.compensationVoltage;     // In pct-output
  private double kVd = 0.126 / ElevatorConstants.compensationVoltage;     // In (pct-output)/(in/s)
  private double kAd = 0.000;      // In (pct-output)/(in/s^2)     2023 = 0.0006
  private double kPd = 0.1;       
  private double kId = 0;    
  private double kDd = 0.0;      // 2023 = 0.02

  /**
   * Creates a new profile generator in disabled mode.
   * @param elevator Elevator subsystem
   * @param log FileLog utility
   */
  public ElevatorProfileGenerator(Elevator elevator, FileLog log) {
    disableProfileControl();
    this.log = log;
    this.elevator = elevator;
  }

  /**
   * Disables the motion profile's control of motors.
   */
  public void disableProfileControl() {
    profileEnabled = false;
  }

  /**
   * Sets the target position for elevator, using motion profile movement. This enables the profiler
   * to take control of the elevator, so only call it if the encoders are working (check interlocks first).
   * @param pos target, in inches (use ElevatorConstants.ElevatorPosition)
   */
  public void setProfileTarget(double pos) {
    profileEnabled = true;
    finalPosition = pos;
    initialPosition = elevator.getElevatorPosition();
    intError = 0;   // Clear integrated error
    prevError = 0;  // Clear previous error
    approachingTarget = false;

    // Save the starting time
    startTime = System.currentTimeMillis();
    lastTime = startTime;

    SmartDashboard.putNumber("ElevatorInitPos", initialPosition);
    SmartDashboard.putNumber("ElevatorTarget", finalPosition);

    currentMPDistance = 0;
    targetMPDistance = Math.abs(finalPosition - initialPosition);
    directionSign = Math.signum(finalPosition - initialPosition);

    // Seed the profile with the current velocity, in case the elevator is already moving
    currentMPVelocity = elevator.getElevatorVelocity() * directionSign;

    log.writeLog(false, "ElevatorProfile", "New Profile", "InitPos", initialPosition , "Target", finalPosition);
  }

  /**
   * Call this method once per scheduler cycle. This method calculates the distance that the elevator should 
   * have traveled at this point in time, per the motion profile. It also calculates velocity in inches/second.
   */

  public void updateProfileCalcs() {
    // The profile has not reached its target, so update the profile velocity and acceleration
    if (currentMPDistance < targetMPDistance) { 
      // Do not continue calculating after the profile *should have* reached its target
      long currentTime = System.currentTimeMillis();
      dt = ((double) (currentTime - lastTime)) / 1000.0;
      lastTime = currentTime;

      double stoppingDistance = 0.5 * currentMPVelocity * currentMPVelocity / stoppingAcceleration;

      // The profile *should be* close enough to the target to be decelerating, so update the acceleratioin
      if (currentMPVelocity >= 0 && (approachingTarget || (targetMPDistance - currentMPDistance) < stoppingDistance) ) {
        approachingTarget = true;
        currentMPAcceleration = -0.5 * currentMPVelocity * currentMPVelocity / (targetMPDistance - currentMPDistance);
        
        // Do not set the acceleration too negative, since that would cause velocity to switch direction later in these calculations
        if (currentMPAcceleration < -currentMPVelocity / dt) {
          currentMPAcceleration = -currentMPVelocity / dt;
        }
      }

      // The profile has a target velocity less than the maximum, so accelerate
      else if (currentMPVelocity < maxVelocity) {
        currentMPAcceleration = maxAcceleration;
      }

      // The profile has a target velocity at the maximum, so do not accelerate
      else {
        currentMPAcceleration = 0;
      }

      // Calculate the target velocity, capped by the maximum
      currentMPVelocity = currentMPVelocity + currentMPAcceleration * dt;
      if (currentMPVelocity > maxVelocity) {
        currentMPVelocity = maxVelocity;
      }

      // Calculate the distance the elevator should have traveled
      currentMPDistance = currentMPDistance + currentMPVelocity * dt;
      if (currentMPDistance > targetMPDistance) {
        currentMPDistance = targetMPDistance;
      }

    // The profile has reached its target, so do not change the theoretical distance and zero the profile velocity and acceleration
    } else {
      currentMPDistance = targetMPDistance;
      currentMPVelocity = 0;
      currentMPAcceleration = 0;
    }
  }

  /**
   * Makes the elevator follow the motion profile. This method should be called exactly once per scheduler cycle.
   * NOTE: Only follow the motion if elevPosControl is true. Otherwise, it is in manual control mode, so do nothing (return 0.0).
   * @return output percent based on calculations, -1.0 to +1.0 (positive = up, negative = down)
   */
  public double trackProfilePeriodic() {
    if (!profileEnabled) return 0.0;

    if (currentMPVelocity > 0 || Math.abs(percentPowerFB) > 0.08) {
      updateElevatorProfileLog(false);
    }

    updateProfileCalcs();
    error = getCurrentPosition() - elevator.getElevatorPosition();
    intError = intError + error * dt;

    // If the elevator is moving up, use the profile terms for moving up
    if (directionSign == 1) {
      percentPowerFF = kFF + kSu * Math.signum(currentMPVelocity * directionSign) + kVu * currentMPVelocity * directionSign + kAu * currentMPAcceleration * directionSign;
      percentPowerFB = kPu * error + ((error - prevError) * kDu) + (kIu * intError);

    // If the elevator is moving down, use the profile terms for moving down
    } else if (directionSign == -1) {
      percentPowerFF = kFF + kSd * Math.signum(currentMPVelocity * directionSign) + kVd * currentMPVelocity * directionSign + kAd * currentMPAcceleration * directionSign;
      percentPowerFB = kPd * error + ((error - prevError) * kDd) + (kId * intError);
    }

    prevError = error;

    // Cap feedback power to prevent jerking the elevator
    percentPowerFB = MathUtil.clamp(percentPowerFB, -0.4, 0.4);

    return percentPowerFF + percentPowerFB;
  }

  /**
   * Writes information about the ElevatorProfileGenerator to the file log.
   * @param logWhenDisabled true = write when robot is disabled, false = only write when robot is enabled
  */
  public void updateElevatorProfileLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, "ElevatorProfile", "Update Calculations",
      "TargetPos", finalPosition,
      "TimeSinceStart", getTimeSinceProfileStart(), "dt", dt,
      "MPPos", getCurrentPosition(), "ActualPos", elevator.getElevatorPosition(),
      "MPVel", (currentMPVelocity * directionSign), "ActualVel", elevator.getElevatorVelocity(),
      "MPAccel", (currentMPAcceleration * directionSign),
      "PowerFF", percentPowerFF, "PowerFB", percentPowerFB);
  }

  /**
   * Gets the current position of the elevator, according to the profile.
   * @return current position, in inches
   */
  public double getCurrentPosition() {
    return currentMPDistance * directionSign + initialPosition;
  }

  /**
   * Gets the target/final position of the elevator, according to the profile.
   * @return target position, in inches
   */
  public double getFinalPosition() {
    return finalPosition;
  }

  /**
   * Gets the velocity of the elevator, according to the profile.
   * @return target velocity, in inches/second
   */
  public double getCurrentVelocity() {
    return currentMPVelocity * directionSign;
  }

  /**
   * Gets the time that has elapsed since the profile started.
   * @return elapsed time, in seconds
   */
  public double getTimeSinceProfileStart() {
    return ((double) (lastTime - startTime)) / 1000.0;
  }
}
