package frc.robot.Subsystems.Intake.Roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
    private final DCMotorSim m_motorSim;

    public RollerIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, RollerConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.rollerAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.rollerVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.rollerPosition = m_motorSim.getAngularPositionRotations();
        inputs.rollerAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setRollerVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
