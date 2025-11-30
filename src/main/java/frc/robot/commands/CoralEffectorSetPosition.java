// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.utilities.DataLogUtil;

public class CoralEffectorSetPosition extends Command {
  private final CoralEffector coralEffector;

  private final boolean autoHold;
  private double position = 0.0;
  private boolean fromShuffleboard;

  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry dLogPosition =
      new DoubleLogEntry(log, "/CoralEffectorSetPosition/Position");
  private final BooleanLogEntry bLogAutoHold =
      new BooleanLogEntry(log, "/CoralEffectorSetPosition/AutoHold");

  /**
   * Sets the position of the coralEffector from Shuffleboard and ends when it is within tolerance.
   *
   * @param autoHold true = automatically adjust position so that both coral sensors detect the
   *     coral. false = hold exact position specified in first parameter.
   * @param coralEffector CoralEffector subsystem
   */
  public CoralEffectorSetPosition(boolean autoHold, CoralEffector coralEffector) {
    this.autoHold = autoHold;
    this.coralEffector = coralEffector;

    this.fromShuffleboard = true;
    addRequirements(coralEffector);

    if (SmartDashboard.getNumber("CoralEffector Goal Position", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("CoralEffector Goal Position", 0);
    }

    // Prime DataLog when robot boots
    long timeNow = RobotController.getFPGATime();
    dLogPosition.append(-1, timeNow);
    bLogAutoHold.append(false, timeNow);
  }

  /**
   * Sets the position of the coralEffector and ends when it is within tolerance.
   *
   * @param position position, in motor rotations
   * @param autoHold true = automatically adjust position so that both coral sensors detect the
   *     coral. false = hold exact position specified in first parameter.
   * @param coralEffector CoralEffector subsystem
   */
  public CoralEffectorSetPosition(double position, boolean autoHold, CoralEffector coralEffector) {
    this.coralEffector = coralEffector;

    this.position = position;
    this.autoHold = autoHold;
    this.fromShuffleboard = false;
    addRequirements(coralEffector);

    // Prime DataLog when robot boots
    long timeNow = RobotController.getFPGATime();
    dLogPosition.append(-1, timeNow);
    bLogAutoHold.append(false, timeNow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO check if elevator is in correct position before running motor

    coralEffector.enableFastLogging(true);

    if (fromShuffleboard) position = SmartDashboard.getNumber("CoralEffector Goal Position", 0.0);
    coralEffector.setCoralEffectorPosition(position, autoHold);

    long timeNow = RobotController.getFPGATime();

    dLogPosition.append(position, timeNow);
    bLogAutoHold.append(autoHold, timeNow);
    DataLogUtil.writeMessage(
        "CoralEffectorSetPosition: Init, Position =", position, ", Auto Hold =", autoHold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEffector.enableFastLogging(false);

    DataLogUtil.writeMessage(
        "CoralEffectorSetPosition: End, Position =",
        position,
        ", Measured Position =",
        coralEffector.getCoralEffectorPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(position - coralEffector.getCoralEffectorPosition())
        < CoralEffectorConstants.centeringTolerance;
  }
}
