// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.Roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake.Roller.RollerIO;
import frc.robot.Subsystems.Intake.Roller.RollerIOInputsAutoLogged;

public class Roller extends SubsystemBase {
  private final RollerIO m_io;
  private final RollerIOInputsAutoLogged m_inputs = new RollerIOInputsAutoLogged();

  public Roller(RollerIO io) {
    System.out.println("[Init] Creating Roller");
    m_io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Roller", m_inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(m_inputs);
  }

  public void setRollerVoltage(double volts) {
    m_io.setRollerVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setRollerPercent(int percent) {
    m_io.setRollerVoltage(((double)percent) / 100 * 12);
  }

  public void runRoller() {
    setRollerPercent(RollerConstants.INTAKING_ROLLER_PERCENT);
  }
}
