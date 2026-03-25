package frc.robot.subsystems.Intake.Drum;

import org.littletonrobotics.junction.AutoLog;

public interface DrumIO {
    @AutoLog
    public static class DrumIOInputs {
        public double drumAppliedVolts = 0;
        public double drumVelocityRadPerSec = 0;
        public double drumPosition = 0;
        public double drumAppliedCurrent = 0;
    }

    public default void updateInputs (DrumIOInputs inputs) {}

    public default void setDrumVoltage (double volts) {}
}
