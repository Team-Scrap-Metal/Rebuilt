package frc.robot.Subsystems.Spindexer;

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

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SpindexerIOSim implements SpindexerIO {
    private final DCMotorSim m_motorSim;

    public SpindexerIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, SpindexerConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.spindexerAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.spindexerVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.spindexerPosition = m_motorSim.getAngularPositionRotations();
        inputs.spindexerAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setSpindexerVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
