package frc.robot.Subsystems.Intake.Drum;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DrumIOSim implements DrumIO {
    private final DCMotorSim m_motorSim;

    public DrumIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, DrumConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(DrumIOInputs inputs) {
        inputs.drumAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.drumVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.drumPosition = m_motorSim.getAngularPositionRotations();
        inputs.drumAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setDrumVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
