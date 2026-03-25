package frc.robot.subsystems.Gyro;

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

public class GyroIOSpark implements GyroIO {
    private final SparkBase gyroMotor;
    private final SparkBase gyroFollowerMotor;
    private final RelativeEncoder gyroEncoder;

    public GyroIOSpark() {
        gyroMotor = new SparkMax(GyroConstants.CAN_ID, MotorType.kBrushless);
        gyroFollowerMotor = new SparkMax(GyroConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);
        gyroEncoder = gyroMotor.getEncoder();

        var motorConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();
        motorConfig
            .inverted(GyroConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(GyroConstants.CURRENT_LIMIT);
        followerConfig
            .apply(motorConfig)
            .follow(gyroMotor, GyroConstants.FOLLOWER_INVERTED);


        gyroMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        gyroFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroAppliedVolts = gyroMotor.getAppliedOutput() * gyroMotor.getBusVoltage();
        inputs.gyroVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(gyroEncoder.getVelocity());
        inputs.gyroPosition = gyroEncoder.getPosition();
        inputs.gyroAppliedCurrent = gyroMotor.getOutputCurrent();
    }

    @Override
    public void setGyroVoltage (double volts) {
        gyroMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
