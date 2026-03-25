package frc.robot.Subsystems.Turret;

import java.io.OutputStream;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretIOSpark implements TurretIO {
    private final SparkBase m_turretMotor;
    private final RelativeEncoder m_turretEncoder;
    private final SparkClosedLoopController m_ClosedLoopController;
    public TurretIOSpark() {
        m_turretMotor = new SparkMax(TurretConstants.CAN_ID, MotorType.kBrushless);
        m_turretEncoder = m_turretMotor.getEncoder();
        m_ClosedLoopController = m_turretMotor.getClosedLoopController();
        var motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(TurretConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurretConstants.CURRENT_LIMIT);
        motorConfig.closedLoop
            .p(TurretConstants.kP)
            .i(TurretConstants.kI)
            .d(TurretConstants.kD)
            .outputRange(TurretConstants.kMinOutput, TurretConstants.kMaxOutput);
        m_turretMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAppliedVolts = m_turretMotor.getAppliedOutput() * m_turretMotor.getBusVoltage();
        inputs.turretVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_turretEncoder.getVelocity());
        inputs.turretPosition = m_turretEncoder.getPosition();
        inputs.turretAppliedCurrent = m_turretMotor.getOutputCurrent();
    }

    @Override
    public void setTurretVoltage (double volts) {
        m_turretMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    @Override
    public void setTurretPosition (float angle) {

    }
}
