package frc.robot.subsystems.Spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SpindexerIOSpark implements SpindexerIO {
    private final SparkBase m_spindexerMotor;
    private final RelativeEncoder m_spindexerEncoder;

    public SpindexerIOSpark() {
        m_spindexerMotor = new SparkMax(SpindexerConstants.CAN_ID, MotorType.kBrushless);
        m_spindexerEncoder = m_spindexerMotor.getEncoder();

        var motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(SpindexerConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(SpindexerConstants.CURRENT_LIMIT);
        m_spindexerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.spindexerAppliedVolts = m_spindexerMotor.getAppliedOutput() * m_spindexerMotor.getBusVoltage();
        inputs.spindexerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_spindexerEncoder.getVelocity());
        inputs.spindexerPosition = m_spindexerEncoder.getPosition();
        inputs.spindexerAppliedCurrent = m_spindexerMotor.getOutputCurrent();
    }

    @Override
    public void setSpindexerVoltage (double volts) {
        m_spindexerMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
