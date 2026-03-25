package frc.robot.subsystems.Spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public double spindexerAppliedVolts = 0;
        public double spindexerVelocityRadPerSec = 0;
        public double spindexerPosition = 0;
        public double spindexerAppliedCurrent = 0;
    }

    public default void updateInputs (SpindexerIOInputs inputs) {}

    public default void setSpindexerVoltage (double volts) {}
}
