// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

    private final MutVoltage m_appliedVoltage;
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle;
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity;

    private final SysIdRoutine m_sysIdRoutine;

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    m_io = io;

      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    m_angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    m_velocity = RadiansPerSecond.mutable(0);

    m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),

        new SysIdRoutine.Mechanism(
            // Apply voltage (units-safe input)
            (volts) -> m_io.setShooterVoltage(volts.in(Volts)),

            // Log data
            (SysIdRoutineLog log) -> {
              log.motor("shooter-wheel")
                  .voltage(m_appliedVoltage.mut_replace(
                      m_inputs.shooterAppliedVolts, Volts))
                  .angularPosition(m_angle.mut_replace(
                      m_inputs.shooterPosition, Radians))
                  .angularVelocity(m_velocity.mut_replace(
                      m_inputs.shooterVelocityRadPerSec, RadiansPerSecond));
            },

            this
        )
    );
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Shooter", m_inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(m_inputs);
  }

  public void setShooterVoltage(double volts) {
    m_io.setShooterVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setShooterPercent(int percent) {
    m_io.setShooterVoltage(((double)percent) / 100 * 12);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command runFullSysId() {
    return Commands.sequence(
        // Quasistatic Forward
        m_sysIdRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(2),

        // Quasistatic Reverse
        m_sysIdRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(2),

        // Dynamic Forward
        m_sysIdRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(2),

        // Dynamic Reverse
        m_sysIdRoutine.dynamic(Direction.kReverse)
    );
}
}
