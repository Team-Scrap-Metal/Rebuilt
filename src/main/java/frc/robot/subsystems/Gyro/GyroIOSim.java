package frc.robot.subsystems.Gyro;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GyroIOSim implements GyroIO {
    private final DCMotorSim m_motorSim;

    public GyroIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, GyroConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.gyroVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.gyroPosition = m_motorSim.getAngularPositionRotations();
        inputs.gyroAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setGyroVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
