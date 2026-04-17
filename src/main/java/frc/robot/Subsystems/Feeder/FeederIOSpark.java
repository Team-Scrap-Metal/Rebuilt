package frc.robot.Subsystems.Feeder;

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

public class FeederIOSpark implements FeederIO {
    private final SparkBase feederMotor;
    private final RelativeEncoder feederEncoder;

    public FeederIOSpark() {
        feederMotor = new SparkMax(FeederConstants.CAN_ID, MotorType.kBrushless);
        feederEncoder = feederMotor.getEncoder();

        var motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(FeederConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(FeederConstants.CURRENT_LIMIT);
        feederMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.feederAppliedVolts = feederMotor.getAppliedOutput() * feederMotor.getBusVoltage();
        inputs.feederVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(feederEncoder.getVelocity());
        inputs.feederPosition = feederEncoder.getPosition();
        inputs.feederAppliedCurrent = feederMotor.getOutputCurrent();
    }

    @Override
    public void setFeederVoltage (double volts) {
        feederMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
