// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Spindexer.SpindexerIOInputsAutoLogged;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  private final LoggedNetworkNumber speedPercentInput = new LoggedNetworkNumber("/Tuning/SpindexerPercent", SpindexerConstants.INDEXING_PERCENT);

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

  public int getTunedPercent() {
    return (int)speedPercentInput.getAsDouble();
  }

  public void runReverse () {
    setSpindexerPercent(-getTunedPercent());
  }
}
