package frc.robot.Subsystems.Turret;

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

    public default void zeroEncoder () {}

    public default void setTurretPosition (double angle) {}

    // public default void setTurretPositionFieldOriented(double ) {}

    public default void setTurretPositionWithController (double joystickX, double joystickY) {}
}
