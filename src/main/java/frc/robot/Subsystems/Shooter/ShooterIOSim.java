package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim m_motorSim;

    public ShooterIOSim() {
        m_motorSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.005, ShooterConstants.GEAR_RATIO),
                DCMotor.getNEO(1));

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shooterAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.shooterVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.shooterPosition = m_motorSim.getAngularPositionRotations();
        inputs.shooterAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setShooterVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
}
