package frc.robot.Subsystems.Feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class FeederIOSim implements FeederIO {
    private final DCMotorSim m_motorSim;

    public FeederIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, FeederConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.feederVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.feederPosition = m_motorSim.getAngularPositionRotations();
        inputs.feederAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setFeederVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}