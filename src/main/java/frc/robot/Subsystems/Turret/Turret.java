// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.Constants;
import frc.robot.Subsystems.Turret.TurretIOInputsAutoLogged;

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
  public void zeroEncoder () {
    m_io.zeroEncoder();
  }

  public Command setTurretPositionWithController(Turret turret, DoubleSupplier joystickX, DoubleSupplier joystickY) {
    return Commands.run(
      () -> {
        double x = joystickX.getAsDouble();
        double y = joystickY.getAsDouble();
        System.out.println("JstickX: " + x + " JstickY: " + y);
        double angle = Math.toDegrees(Math.atan2(y, x));
        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        System.out.println("Angle: " + angle + "\n");
        System.out.println("Magnitude: " + magnitude + "\n");
        if(magnitude > 0.1) {
          setTurretPosition(angle);
        }
      },
      turret
      );
  }

  // public void targetHub (Pose2d pose) {
    
  // }
}
