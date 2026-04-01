// package frc.robot.subsystems.Intake.Pivot;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// public class PivotIOSim implements PivotIO {
//     private final DCMotorSim m_motorSim;

//     public PivotIOSim() {
//         m_motorSim =
//             new DCMotorSim(
//                 LinearSystemId.createDCMotorSystem(
//                 DCMotor.getNEO(1), 0.005, PivotConstants.GEAR_RATIO),
//                 DCMotor.getNEO(1));

//     }

//     @Override
//     public void updateInputs(PivotIOInputs inputs) {
//         inputs.pivotAppliedVolts = m_motorSim.getInputVoltage(); 
//         inputs.pivotVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
//         inputs.pivotPosition = m_motorSim.getAngularPositionRotations();
//         inputs.pivotAppliedCurrent = m_motorSim.getCurrentDrawAmps();
//     }

//     @Override
//     public void setPivotVoltage (double volts) {
//         m_motorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
//     }
// }
