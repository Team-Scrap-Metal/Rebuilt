package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSpark implements ShooterIO {
    private final SparkBase m_shooterMotor;
    private final SparkBase m_shooterFollowerMotor;
    private final RelativeEncoder m_shooterEncoder;

    public ShooterIOSpark() {
        m_shooterMotor = new SparkMax(ShooterConstants.CAN_ID, MotorType.kBrushless);
        m_shooterFollowerMotor = new SparkMax(ShooterConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);
        m_shooterEncoder = m_shooterMotor.getEncoder();

        var motorConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();
        motorConfig
            .inverted(ShooterConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
            .closedLoop
                .p(ShooterConstants.KP)
                .i(ShooterConstants.KI)
                .d(ShooterConstants.KD)
                .outputRange(ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT)
            .feedForward
                .kV(ShooterConstants.KV);
        followerConfig
            .apply(motorConfig)
            .follow(m_shooterMotor, ShooterConstants.FOLLOWER_INVERTED);
        
        m_shooterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = m_shooterMotor.getAppliedOutput() * m_shooterMotor.getBusVoltage();
        inputs.shooterVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_shooterEncoder.getVelocity());
        inputs.shooterPosition = m_shooterEncoder.getPosition();
        inputs.shooterAppliedCurrent = m_shooterMotor.getOutputCurrent();
    }

    @Override
    public void setShooterVoltage (double volts) {
        m_shooterMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override 
    public void setShooterRPM (double rpm) {
        m_shooterMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }
}
