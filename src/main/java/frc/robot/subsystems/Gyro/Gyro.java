// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gyro;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.Gyro.GyroConstants.GyroState;

public class Gyro extends SubsystemBase {
  private final GyroIO m_io;
  private final GyroIOInputsAutoLogged m_inputs = new GyroIOInputsAutoLogged();

  public Gyro(GyroIO io) {
    System.out.println("[Init] Creating Gyro");
    m_io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Gyro", m_inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(m_inputs);
  }

  public void setGyroVoltage(double volts) {
    m_io.setGyroVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setGyroPercent(int percent) {
    m_io.setGyroVoltage(((double)percent) / 100 * 12);
  }

  // public void runGyro () {
  //   switch (GyroConstants.currentState) {
  //     case INTAKING:
  //       setGyroPercent(GyroConstants.INTAKING_gyro_PERCENT);
  //       break;
  //     case LAUNCHING:
  //       setGyroPercent(GyroConstants.LAUNCHING_gyro_PERCENT);
  //       break;
  //     default:
  //       setGyroPercent(GyroConstants.INTAKING_gyro_PERCENT);
  //   }
  // }

  public void gyroIntake() {
    setGyroPercent(GyroConstants.INTAKING_gyro_PERCENT);
  }

  public void gyroLaunch() {
    setGyroPercent(GyroConstants.LAUNCHING_gyro_PERCENT);
  }
}
