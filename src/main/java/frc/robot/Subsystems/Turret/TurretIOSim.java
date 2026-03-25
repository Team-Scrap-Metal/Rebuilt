package frc.robot.Subsystems.Turret;

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
import com.revrobotics.spark.SparkClosedLoopController;

public class TurretIOSim implements TurretIO {
    private final DCMotorSim m_motorSim;
    /* private final SparkMaxConfig m_Config;
    private final SparkClosedLoopController m_ClosedLoopController; */

    public TurretIOSim() {
        m_motorSim =
            new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.005, TurretConstants.GEAR_RATIO),
            DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretAppliedVolts = m_motorSim.getInputVoltage(); 
        inputs.turretVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
        inputs.turretPosition = m_motorSim.getAngularPositionRotations();
        inputs.turretAppliedCurrent = m_motorSim.getCurrentDrawAmps();
    }

    @Override
    public void setTurretVoltage (double volts) {
        m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }
    @Override
    public void setTurretPosition(float angle) {
       m_motorSim.setAngle(angle); 
    }
}
