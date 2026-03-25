package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterAppliedVolts = 0;
        public double shooterVelocityRadPerSec = 0;
        public double shooterPosition = 0;
        public double shooterAppliedCurrent = 0;
    }

    public default void updateInputs (ShooterIOInputs inputs) {}

    public default void setShooterVoltage (double volts) {}
}
