// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Turret;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Turret.TurretIOInputsAutoLogged;

public class Turret extends SubsystemBase {
  private final TurretIO m_io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    System.out.println("[Init] Creating Turret");
    this.m_io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Turret", inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(inputs);
  }

  public void setTurretVoltage(double volts) {
    m_io.setTurretVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setTurretPercent(int percent) {
    m_io.setTurretVoltage(((double)percent) / 100 * 12);
  }
}
