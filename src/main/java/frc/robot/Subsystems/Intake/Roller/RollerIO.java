package frc.robot.Subsystems.Intake.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double rollerAppliedVolts = 0;
        public double rollerVelocityRadPerSec = 0;
        public double rollerPosition = 0;
        public double rollerAppliedCurrent = 0;
    }

    public default void updateInputs (RollerIOInputs inputs) {}

    public default void setRollerVoltage (double volts) {}
}
