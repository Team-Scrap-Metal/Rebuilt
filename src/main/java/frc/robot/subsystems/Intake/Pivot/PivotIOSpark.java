package frc.robot.Subsystems.Intake.Pivot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Shooter.ShooterConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class PivotIOSpark implements PivotIO {
    private final SparkBase pivotMotor;
    private final SparkBase pivotFollowerMotor;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotClosedLoopController;
    private final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/PivotKP", PivotConstants.Kv);
    private final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/PivotKI", PivotConstants.Kv);
    private final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/PivotKD", PivotConstants.Kv);
    private final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/PivotKS", PivotConstants.Kv);
    private final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/PivotKV", PivotConstants.Kv);
    private final LoggedNetworkNumber target = new LoggedNetworkNumber("/Tuning/TargetAngle", PivotConstants.PIVOT_STARTING_ANGLE);

    private double lastKP = ShooterConstants.KP;
    private double lastKI = ShooterConstants.KI;
    private double lastKD = ShooterConstants.KD;
    private double lastKS = ShooterConstants.KS;
    private double lastKV = ShooterConstants.KV;

    public PivotIOSpark() {
        pivotMotor = new SparkMax(PivotConstants.CAN_ID, MotorType.kBrushless);
        pivotFollowerMotor = new SparkMax(PivotConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotClosedLoopController = pivotMotor.getClosedLoopController();

        var motorConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();
        motorConfig
            .inverted(PivotConstants.INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(PivotConstants.CURRENT_LIMIT);
        motorConfig
            .closedLoop
                .p(PivotConstants.Kp)
                .i(PivotConstants.Ki)
                .d(PivotConstants.Kd)
            .feedForward 
                .kS(PivotConstants.Ks)
                .kV(PivotConstants.Kv);
        followerConfig
            .apply(motorConfig)
            .follow(pivotMotor, PivotConstants.FOLLOWER_INVERTED);

        pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotAppliedCurrent = pivotMotor.getOutputCurrent();

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

            pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastKP = newKP;
            lastKI = newKI;
            lastKD = newKD;
            lastKS = newKS;
            lastKV = newKV;
            }
    }

    @Override
    public void setPivotVoltage (double volts) {
        pivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    @Override
    public void setPivotPosition (double angle) {
        pivotClosedLoopController.setSetpoint(target.getAsDouble(), ControlType.kPosition);
        System.out.println("Rotating Arm To: " + angle);
    }
}
