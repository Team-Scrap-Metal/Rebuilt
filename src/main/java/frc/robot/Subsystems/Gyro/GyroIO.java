package frc.robot.subsystems.Gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public double gyroAppliedVolts = 0;
        public double gyroVelocityRadPerSec = 0;
        public double gyroPosition = 0;
        public double gyroAppliedCurrent = 0;
    }

    public default void updateInputs (GyroIOInputs inputs) {}

    public default void setGyroVoltage (double volts) {}
}
