package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double turretAppliedVolts = 0;
        public double turretVelocityRadPerSec = 0;
        public double turretPosition = 0;
        public double turretAppliedCurrent = 0;
    }

    public default void updateInputs (TurretIOInputs inputs) {}

    public default void setTurretVoltage (double volts) {}

    public default void setTurretPosition (float angle) {}
}
