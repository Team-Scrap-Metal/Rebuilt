<<<<<<< HEAD:src/main/java/frc/robot/subsystems/Turret/TurretIOSpark.java
  package frc.robot.subsystems.Turret;
=======
package frc.robot.Subsystems.Turret;
>>>>>>> DRIVE-TECH:src/main/java/frc/robot/Subsystems/Turret/TurretIOSpark.java

import java.io.OutputStream;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Shooter.ShooterConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretIOSpark implements TurretIO {
    private final SparkBase m_turretMotor;
    private final SparkRelativeEncoder m_turretEncoder;
    private final SparkClosedLoopController m_turretClosedLoopController;
  
    private final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Turret/TurretKP", TurretConstants.kP);
    private final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Turret/TurretKI", TurretConstants.kI);
    private final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Turret/TurretKD", TurretConstants.kD);
    private final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Turret/TurretKS", TurretConstants.kS);
    private final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Turret/TurretKV", TurretConstants.kV);
    private final LoggedNetworkNumber Target_Pose = new LoggedNetworkNumber("/Tuning/Turret/SetPoint", 0);
    private double lastKP = ShooterConstants.KP;
    private double lastKI = ShooterConstants.KI;
    private double lastKD = ShooterConstants.KD;
    private double lastKS = ShooterConstants.KS;
    private double lastKV = ShooterConstants.KV;

    public TurretIOSpark() {
        m_turretMotor = new SparkMax(TurretConstants.CAN_ID, MotorType.kBrushless);
        m_turretEncoder = (SparkRelativeEncoder) m_turretMotor.getEncoder();
        m_turretClosedLoopController = m_turretMotor.getClosedLoopController();
    
        var motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(TurretConstants.INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurretConstants.CURRENT_LIMIT)
        .closedLoop
            .p(TurretConstants.kP)
            .i(TurretConstants.kI)
            .d(TurretConstants.kD)
            .outputRange(TurretConstants.kMinOutput, TurretConstants.kMaxOutput)
        .feedForward
            .kS(TurretConstants.kS)
            .kV(TurretConstants.kV);
        motorConfig
        .encoder
            .positionConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR);
        motorConfig
        .softLimit
            .forwardSoftLimit(TurretConstants.FORWARD_SOFT_LIMIT)
            .forwardSoftLimitEnabled(TurretConstants.FORWARD_SOFT_LIMIT_ENABLED)
            .reverseSoftLimit(TurretConstants.BACKWARD_SOFT_LIMIT)
            .reverseSoftLimitEnabled(TurretConstants.BACKWARD_SOFT_LIMIT_ENABLED);

        m_turretMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
        
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAppliedVolts = m_turretMotor.getAppliedOutput() * m_turretMotor.getBusVoltage();
        inputs.turretVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(m_turretEncoder.getVelocity());
        inputs.turretPosition = Units.radiansToDegrees(m_turretEncoder.getPosition());
        inputs.turretAppliedCurrent = m_turretMotor.getOutputCurrent();
        double newKP = kP.get();
        double newKI = kI.get();
        double newKD = kD.get();
        double newKS = kS.get();
        double newKV = kV.get();
         if (newKP != lastKP || newKI != lastKI || newKD != lastKD || newKS != lastKS || newKV != lastKV) {

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
            m_turretMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            lastKP = newKP;
            lastKI = newKI;
            lastKD = newKD;
            lastKS = newKS;
            lastKV = newKV;
        }
    
    }

    @Override
    public void setTurretVoltage (double volts) {
        m_turretMotor.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    @Override
    public void setTurretPosition (double angle) {
        System.out.println("Turret position set to: " + angle);
        m_turretClosedLoopController.setSetpoint(angle, ControlType.kPosition);
    }
    @Override
    public void zeroEncoder() {
        m_turretEncoder.setPosition(0);
    }
}
