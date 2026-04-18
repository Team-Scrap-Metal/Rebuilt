// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Feeder;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final LoggedNetworkNumber speedPercentInput = new LoggedNetworkNumber("/Tuning/FeederPercent", FeederConstants.FEEDING_PERCENT);

  public Feeder(FeederIO io) {
    System.out.println("[Init] Creating Feeder");
    this.io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Feeder", inputs);
  }

  public void updateInputs () {
    io.updateInputs(inputs);
  }

  public void setFeederVoltage(double volts) {
    io.setFeederVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setFeederPercent(int percent) {
    io.setFeederVoltage(((double)percent) / 100 * 12);
  }

  public int getTunedPercent() {
    return (int)speedPercentInput.getAsDouble();
  }

  public void runReverse () {
    setFeederPercent(-getTunedPercent());
  }

}
