// package frc.robot.subsystems.Intake.Pivot;

// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.util.Units;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// public class PivotIOSpark implements PivotIO {
//     private final SparkBase pivotMotor;
//     private final SparkBase pivotFollowerMotor;
//     private final RelativeEncoder pivotEncoder;

//     public PivotIOSpark() {
//         pivotMotor = new SparkMax(PivotConstants.CAN_ID, MotorType.kBrushless);
//         pivotFollowerMotor = new SparkMax(PivotConstants.FOLLOWER_CAN_ID, MotorType.kBrushless);
//         pivotEncoder = pivotMotor.getEncoder();

//         var motorConfig = new SparkMaxConfig();
//         var followerConfig = new SparkMaxConfig();
//         motorConfig
//             .inverted(PivotConstants.INVERTED)
//             .idleMode(IdleMode.kCoast)
//             .smartCurrentLimit(PivotConstants.CURRENT_LIMIT);
//         followerConfig
//             .apply(motorConfig)
//             .follow(pivotMotor, PivotConstants.FOLLOWER_INVERTED);


//         pivotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         pivotFollowerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     @Override
//     public void updateInputs(PivotIOInputs inputs) {
//         inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
//         inputs.pivotVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(pivotEncoder.getVelocity());
//         inputs.pivotPosition = pivotEncoder.getPosition();
//         inputs.pivotAppliedCurrent = pivotMotor.getOutputCurrent();
//     }

//     @Override
//     public void setPivotVoltage (double volts) {
//         pivotMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
//     }
// }
