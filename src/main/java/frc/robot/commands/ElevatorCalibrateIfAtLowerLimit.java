// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.DataLogUtil;

public class ElevatorCalibrateIfAtLowerLimit extends Command {
  private final Elevator elevator;
  

  /**
   * If the elevator is at the lower limit, calibrate the encoders.
   * This is a blocking call and will wait up to 200ms for the zero to apply.
   * @param elevator Elevator subsystem
   */
  public ElevatorCalibrateIfAtLowerLimit(Elevator elevator) {
    this.elevator = elevator;
    
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.checkAndCalibrateEncoders();
    DataLogUtil.writeMessage("ElevatorCalibrateIfAtLowerLimit: Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // Returns whether the command should be able to run while the robot is disabled (default = false). 
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
