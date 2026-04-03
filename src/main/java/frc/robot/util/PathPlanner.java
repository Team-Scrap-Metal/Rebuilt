// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Subsystems.Drive.DriveConstants.ppConfig;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.Drive;

/** Add your docs here. */
public class PathPlanner extends SubsystemBase {
  private Drive drive;
  private PoseEstimator pose;


  public PathPlanner(Drive drive, PoseEstimator pose) {
    this.drive = drive;
    this.pose = pose;


    SmartDashboard.putString("Running PathPlanner", "running");
    AutoBuilder.configure(
        pose::getCurrentPose2d,
        pose::resetPose,
        drive::getChassisSpeeds,
        drive::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
              5.0, 
              0.0, 
              0.0), 
            new PIDConstants(
              5.0, 
              0.0, 
              0.0)
              ),
        ppConfig,
        () -> {
          if (Constants.getAlliance().isPresent()) {
            return Constants.getAlliance().get() == DriverStation.Alliance.Red;
          } else {
            return false;
          }
        },
        drive);
    
            PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }
}