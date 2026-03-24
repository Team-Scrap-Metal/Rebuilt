package frc.robot.subsystems.Feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double feederAppliedVolts = 0;
        public double feederVelocityRadPerSec = 0;
        public double feederPosition = 0;
        public double feederAppliedCurrent = 0;
    }

    public default void updateInputs (FeederIOInputs inputs) {}

    public default void setFeederVoltage (double volts) {}
}
