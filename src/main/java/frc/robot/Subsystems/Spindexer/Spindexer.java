// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spindexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public Spindexer(SpindexerIO io) {
    System.out.println("[Init] Creating Spindexer");
    this.io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Spindexer", inputs);
  }

  public void updateInputs () {
    io.updateInputs(inputs);
  }

  public void setSpindexerVoltage(double volts) {
    io.setSpindexerVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setSpindexerPercent(int percent) {
    io.setSpindexerVoltage(((double)percent) / 100 * 12);
  }
}
