// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.Pivot;
// import frc.robot.Subsystems.Intake.Pivot.PivotIO;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Pivot extends SubsystemBase {
  private final PivotIO m_io;
  private final PivotIOInputsAutoLogged m_inputs = new PivotIOInputsAutoLogged();
  
  public Pivot(PivotIO io) {
    System.out.println("[Init] Creating Pivot");
    m_io = io;
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Pivot", m_inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(m_inputs);
  }

  public void setPivotVoltage(double volts) {
    m_io.setPivotVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setPivotPercent(int percent) {
    m_io.setPivotVoltage(((double)percent) / 100 * 12);
  }

  public void setPivotPosition (double angle) {
    m_io.setPivotPosition(angle);
  }

  // public void runPivot () {
  //   switch (PivotConstants.currentState) {
  //     case INTAKING:
  //       setPivotPercent(PivotConstants.INTAKING_DRUM_PERCENT);
  //       break;
  //     case LAUNCHING:
  //       setPivotPercent(PivotConstants.LAUNCHING_DRUM_PERCENT);
  //       break;
  //     default:
  //       setPivotPercent(PivotConstants.INTAKING_DRUM_PERCENT);
  //   }
  // }

 
}
