// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final TurretIO m_io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final LoggedNetworkNumber something = new LoggedNetworkNumber("/Tuning/Turret/something", 200);

  private final MutVoltage m_appliedVoltage;
  private final MutAngle m_angle;


  public Turret (TurretIO io) {
    System.out.println("[Init] Creating Turret");
    this.m_io = io;
    m_appliedVoltage = Volts.mutable(0);
    m_angle = Radians.mutable(0);
    
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
  public void setTurretPosition(double angle) {
    m_io.setTurretPosition(angle);
  }

  public Command setTurretPositionWithController(double joystickX, double joystickY) {
    return Commands.run(
      () -> {
        double angle =  Math.atan2(joystickY, joystickX);
        double magnitude = Math.pow(joystickX, 2) + Math.pow(joystickY, joystickX);

        if (magnitude > .2) {
          setTurretPosition(angle);
        }
      });
  }
}
