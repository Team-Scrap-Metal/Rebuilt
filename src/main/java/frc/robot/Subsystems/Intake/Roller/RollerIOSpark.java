package frc.robot.Subsystems.Intake.Roller;

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

public class RollerIOSpark implements RollerIO {
    private final SparkBase rollerMotor;
    private final RelativeEncoder rollerEncoder;

    public RollerIOSpark() {
        rollerMotor = new SparkMax(RollerConstants.CAN_ID, MotorType.kBrushless);
        rollerEncoder = rollerMotor.getEncoder();

        var motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(RollerConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(RollerConstants.CURRENT_LIMIT);
        rollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.rollerAppliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
        inputs.rollerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rollerEncoder.getVelocity());
        inputs.rollerPosition = rollerEncoder.getPosition();
        inputs.rollerAppliedCurrent = rollerMotor.getOutputCurrent();
    }

    @Override
    public void setRollerVoltage (double volts) {
        rollerMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
