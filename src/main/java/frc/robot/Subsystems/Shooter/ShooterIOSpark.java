package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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

    private final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/ShooterKP", ShooterConstants.KP);
    private final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/ShooterKI", ShooterConstants.KI);
    private final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/ShooterKD", ShooterConstants.KD);
    private final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/ShooterKS", ShooterConstants.KS);
    private final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/ShooterKV", ShooterConstants.KV);

    private double lastKP = ShooterConstants.KP;
    private double lastKI = ShooterConstants.KI;
    private double lastKD = ShooterConstants.KD;
    private double lastKS = ShooterConstants.KS;
    private double lastKV = ShooterConstants.KV;

    private double m_shooterVelocityRPM = 0;
    
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
                // .allowedClosedLoopError(ShooterConstants.PID_TOLERANCE, ClosedLoopSlot.kSlot0)
            .feedForward
                .kS(ShooterConstants.KS)
                .kV(ShooterConstants.KV);
                // .kA(ShooterConstants.KA);
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
        inputs.shooterVelocityRPM = m_shooterEncoder.getVelocity();
        inputs.shooterPosition = m_shooterEncoder.getPosition();
        inputs.shooterAppliedCurrent = m_shooterMotor.getOutputCurrent();

        m_shooterVelocityRPM = inputs.shooterVelocityRPM;
        double newKP = kP.get();
        double newKI = kI.get();
        double newKD = kD.get();
        double newKS = kS.get();
        double newKV = kV.get();

        if (newKP != lastKP || newKI != lastKI || newKD != lastKD 
            || newKS != lastKS || newKV != lastKV) {

            var config = new SparkMaxConfig();

            config
                .closedLoop
                    .p(newKP)
                    .i(newKI)
                    .d(newKD)
                    .outputRange(ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT)
                .feedForward
                    .kS(newKS)
                    .kV(newKV);
                    // .kA(ShooterConstants.KA);

            m_shooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastKP = newKP;
            lastKI = newKI;
            lastKD = newKD;
            lastKS = newKS;
            lastKV = newKV;
        }
    }

    @Override
    public void setShooterVoltage (double volts) {
        m_shooterMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override 
    public void setShooterRPM (double rpm) {
        System.out.println("Shooter rpm set to: " + rpm);
        m_shooterMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }
 
    @Override
    public double getShooterRPM() {
        return m_shooterVelocityRPM;
    }
}
