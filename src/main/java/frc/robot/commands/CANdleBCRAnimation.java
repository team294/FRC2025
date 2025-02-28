// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class CANdleBCRAnimation extends Command {
  private final LED led;
  private final FileLog log;
  private int total;
  private float offset = 0;
  private boolean fromShuffleboard;

  private Color orange = new Color(255, 100, 0);
  private Color blue = new Color(0, 0, 255);

  public CANdleBCRAnimation(LED led, FileLog log) {
    this.led = led;
    this.log = log;
    fromShuffleboard = true;
    SmartDashboard.putNumber("NumberLEDs", 67);
    addRequirements(led);
  }

  // t is from -1.0 to 1.0
  private Color lerpColor(Color color1, Color color2, double t) {
    double r, g, b;
    double _t = (t + 1) / 2;
    r = (color2.red - color1.red) * _t + color1.red;
    g = (color2.green - color1.green) * _t + color1.green;
    b = (color2.blue - color1.blue) * _t + color1.blue;
    return new Color(r, g, b);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO utilize fromShuffleboard boolean
    total = (int) SmartDashboard.getNumber("NumberLEDs", 1) - 1;
    led.setLEDs(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    offset += 420.0 / 690.0;
    for (int i = 0; i < total; i++) {
      double sin = Math.sin((i + offset) * 0.5);
      Color color = lerpColor(orange, blue, sin);
      led.setLEDs(color, 0, i);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.setLEDs(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
