// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Gyro;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
/**ALL THE LOGGABLE INPUTS AND OUTPUTS OF THE GYRO IN EVERY MODE*/
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {

    public Rotation2d anglePositionRad = new Rotation2d();

    public Rotation2d rollPositionRad = new Rotation2d();
    public Rotation2d pitchPositionRad = new Rotation2d();
    public Rotation2d yawPositionRad = new Rotation2d();
   
    /**GET ANGULAR VELOCITY OF THE ROLL*/
    public double rollVelocityRadPerSec = 0.0; 
    /**GET ANGULAR VELOCITY OF THE PITCH*/
    public double pitchVelocityRadPerSec = 0.0; 
    /**GET ANGULAR VELOCITY OF THE YAW*/
    public double yawVelocityRadPerSec = 0.0; 

    public double temperatureCelcius = 0.0;
    public double ratePositionRadPerSec = 0.0;
  }

  /** UPDATES THE LOGS */
  public default void updateInputs(GyroIOInputs inputs) {}

  /** RESETS THE HEADING */
  public default void zeroHeading() {}
}