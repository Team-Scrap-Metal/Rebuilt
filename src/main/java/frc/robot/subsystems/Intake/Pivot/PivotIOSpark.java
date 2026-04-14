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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Shooter.ShooterConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class PivotIOSpark implements PivotIO {
    private final SparkBase pivotMotor;
    private final SparkBase pivotFollowerMotor;
    private final RelativeEncoder pivotEncoder;
    // private final SparkClosedLoopController pivotClosedLoopController;

    private ArmFeedforward ff;
    private ProfiledPIDController pid;

    private final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/PivotKP", PivotConstants.Kp);
    private final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/PivotKI", PivotConstants.Ki);
    private final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/PivotKD", PivotConstants.Kd);
    private final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/PivotKS", PivotConstants.Ks);
    private final LoggedNetworkNumber kG = new LoggedNetworkNumber("/Tuning/PivotKG", PivotConstants.Kg);
    private final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/PivotKV", PivotConstants.Kv);
    private final LoggedNetworkNumber target = new LoggedNetworkNumber("/Tuning/TargetAngle", PivotConstants.PIVOT_STARTING_ANGLE);

    private double lastKP = PivotConstants.Kp;
    private double lastKI = PivotConstants.Ki;
    private double lastKD = PivotConstants.Kd;
    private double lastKS = PivotConstants.Ks;
    private double lastKG = PivotConstants.Kg;
    private double lastKV = PivotConstants.Kv;

    private double pivotPosition;

    public PivotIOSpark() {
        pivotMotor = new SparkMax(PivotConstants.CAN_ID, MotorType.kBrushless);
        pivotFollowerMotor = new SparkMax(PivotConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        // pivotClosedLoopController = pivotMotor.getClosedLoopController();

        var motorConfig = new SparkMaxConfig();
        var followerConfig = new SparkMaxConfig();
        motorConfig
            .inverted(PivotConstants.INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(PivotConstants.CURRENT_LIMIT)
            .encoder.positionConversionFactor(1/PivotConstants.GEAR_RATIO)
                .velocityConversionFactor(1/PivotConstants.GEAR_RATIO);
                
        pivotEncoder.setPosition(0);
                    
        // motorConfig
        //     .closedLoop
        //         .p(PivotConstants.Kp)
        //         .i(PivotConstants.Ki)
        //         .d(PivotConstants.Kd)
        //     .feedForward 
        //         .kS(PivotConstants.Ks)
        //         .kV(PivotConstants.Kv);
        followerConfig
            .apply(motorConfig)
            .follow(pivotMotor, PivotConstants.FOLLOWER_INVERTED);

        pid = new ProfiledPIDController(
            lastKP, 
            lastKI, 
            lastKD,
            new Constraints(PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCELERATION));
        ff = new ArmFeedforward(lastKS, lastKG, lastKV);

        pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
        inputs.pivotPosition = Units.rotationsToDegrees(pivotEncoder.getPosition());
        inputs.pivotAppliedCurrent = pivotMotor.getOutputCurrent();

        pivotPosition = inputs.pivotPosition;

        double newKP = kP.get();
        double newKI = kI.get();
        double newKD = kD.get();
        double newKS = kS.get();
        double newKG = kG.get();
        double newKV = kV.get();

        if (newKP != lastKP || newKI != lastKI || newKD != lastKD) { 
            pid.setPID(newKP, newKI, newKD);            
            lastKP = newKP;
            lastKI = newKI;
            lastKD = newKD;
        }
        if (newKS != lastKS || newKG != lastKG || newKV != lastKV) {
            ff.setKg(newKG);
            ff.setKs(newKS);
            ff.setKv(newKV);
            lastKS = newKS;
            lastKG = newKG;
            lastKV = newKV;
        }
    }

    @Override
    public void setPivotVoltage (double volts) {
        pivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    @Override
    public void setPivotPosition (double angle) {
        pivotMotor.setVoltage(
            pid.calculate(pivotPosition, target.getAsDouble()) +
            ff.calculate(target.getAsDouble(), pid.getSetpoint().velocity)
        );
        System.out.println("Rotating Arm To: " + angle);
    }
}
