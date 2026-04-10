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
  private boolean m_passiveToggled = false;
  
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

  /**
   * Run pivot at default speeds
   * @param up Set to true to run pivot up
   */
  public void runPivot (boolean up) {
    setPivotPercent(up ? PivotConstants.PIVOTING_PERCENT : -PivotConstants.PIVOTING_PERCENT);
  }

  public void togglePassiveDown () {
    m_passiveToggled = !m_passiveToggled;
    setPivotPercent(m_passiveToggled ? -PivotConstants.PASSIVE_PERCENT : 0);
  }
}
