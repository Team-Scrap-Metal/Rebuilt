// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Drum;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Drum.DrumIO;
import frc.robot.subsystems.Intake.Drum.DrumIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Drum.DrumConstants.DrumState;

public class Drum extends SubsystemBase {
  private final DrumIO m_io;
  private final DrumIOInputsAutoLogged m_inputs = new DrumIOInputsAutoLogged();
  private final LoggedNetworkNumber speedLaunchPercentInput = new LoggedNetworkNumber("/Tuning/DrumLaunchPercent", DrumConstants.LAUNCHING_DRUM_PERCENT);
  private final LoggedNetworkNumber speedIntakePercentInput = new LoggedNetworkNumber("/Tuning/DrumIntakePercent", DrumConstants.INTAKING_DRUM_PERCENT);

  public Drum(DrumIO io) {
    System.out.println("[Init] Creating Drum");
    m_io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Drum", m_inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(m_inputs);
  }

  public void setDrumVoltage(double volts) {
    m_io.setDrumVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setDrumPercent(int percent) {
    m_io.setDrumVoltage(((double)percent) / 100 * 12);
  }

  // public void runDrum () {
  //   switch (DrumConstants.currentState) {
  //     case INTAKING:
  //       setDrumPercent(DrumConstants.INTAKING_DRUM_PERCENT);
  //       break;
  //     case LAUNCHING:
  //       setDrumPercent(DrumConstants.LAUNCHING_DRUM_PERCENT);
  //       break;
  //     default:
  //       setDrumPercent(DrumConstants.INTAKING_DRUM_PERCENT);
  //   }
  // }

  public void drumIntake() {
    setDrumPercent(getTunedIntakePercent());
  }

  public void drumLaunch() {
    setDrumPercent(getTunedLaunchPercent());
  }

  public int getTunedIntakePercent() {
    return (int)speedIntakePercentInput.getAsDouble();
  }

  public int getTunedLaunchPercent() {
    return (int)speedLaunchPercentInput.getAsDouble();
  }
}
