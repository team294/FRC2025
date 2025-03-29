// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeGrabber;


public class AlgaeSetShotMode extends InstantCommand {
  AlgaeGrabber algaeGrabber;
  boolean isStandard;

  /**
   * 
   * @param algaeGrabber
   */
  public AlgaeSetShotMode(AlgaeGrabber algaeGrabber, boolean isStandard) {
    this.algaeGrabber = algaeGrabber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeGrabber.setAlgaeShotMode(isStandard);
  }
}
