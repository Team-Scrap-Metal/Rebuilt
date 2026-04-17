package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterAppliedVolts = 0;
        public double shooterVelocityRadPerSec = 0;
        public double shooterVelocityRPM = 0;
        public double shooterPosition = 0;
        public double shooterAppliedCurrent = 0;
    }

    public default void updateInputs (ShooterIOInputs inputs) {}

    public default void setShooterVoltage (double volts) {}

    public default void setShooterRPM (double rpm) {}

    public default double getShooterRPM () {return 0.0;}
}
