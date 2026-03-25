package frc.robot.subsystems.Intake.Drum;

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
    private final SparkBase drumMotor;
    private final SparkBase drumFollowerMotor;
    private final RelativeEncoder drumEncoder;

    public GyroIOSpark() {
        drumMotor = new SparkMax(GyroConstants.CAN_ID, MotorType.kBrushless);
        drumFollowerMotor = new SparkMax(GyroConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);
        drumEncoder = drumMotor.getEncoder();

        var motorConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();
        motorConfig
            .inverted(GyroConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(GyroConstants.CURRENT_LIMIT);
        followerConfig
            .apply(motorConfig)
            .follow(drumMotor, GyroConstants.FOLLOWER_INVERTED);


        drumMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        drumFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(DrumIOInputs inputs) {
        inputs.drumAppliedVolts = drumMotor.getAppliedOutput() * drumMotor.getBusVoltage();
        inputs.drumVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(drumEncoder.getVelocity());
        inputs.drumPosition = drumEncoder.getPosition();
        inputs.drumAppliedCurrent = drumMotor.getOutputCurrent();
    }

    @Override
    public void setDrumVoltage (double volts) {
        drumMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
