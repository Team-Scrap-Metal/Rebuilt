package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim m_motorSim;

    public ShooterIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, ShooterConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.shooterVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.shooterPosition = m_motorSim.getAngularPositionRotations();
        inputs.shooterAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setShooterVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
