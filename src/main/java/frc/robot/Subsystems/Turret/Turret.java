// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Turret;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import frc.robot.Constants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Turret.TurretIOInputsAutoLogged;
import frc.robot.Subsystems.Turret.TurretConstants.*;

public class Turret extends SubsystemBase {
  private final TurretIO m_io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final LoggedNetworkNumber something = new LoggedNetworkNumber("/Tuning/Turret/something", 200);
  
  private boolean staticSetpointOverridesTurret;
  private boolean manualControlToggle;
  private double angleStaticSetpoint;

  private boolean m_turretEnabled;

  private final MutVoltage m_appliedVoltage;
  private final MutAngle m_angle;

  public Turret (TurretIO io) {
    System.out.println("[Init] Creating Turret");
    this.m_io = io;
    m_appliedVoltage = Volts.mutable(0);
    m_angle = Radians.mutable(0);

    manualControlToggle = TurretConstants.TURRET_DEFAULT_MANUAL_CONTROL;
    Logger.recordOutput("Turret/ManualControlToggled", manualControlToggle);
    staticSetpointOverridesTurret = true;
    Logger.recordOutput("Turret/StaticSetpointOverridesTurret", staticSetpointOverridesTurret);

    m_turretEnabled = TurretConstants.TURRET_ENABLED_DEFAULT;
    Logger.recordOutput("Turret/Enabled", m_turretEnabled);
  }

  @Override
  public void periodic() {
    this.updateInputs();
    Logger.processInputs("Turret", inputs);
  }

  public void updateInputs () {
    m_io.updateInputs(inputs);
  }

  public void setTurretVoltage(double volts) {
    m_io.setTurretVoltage(
      MathUtil.clamp(volts, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE)
    );
  }

  public void setTurretPercent(int percent) {
    m_io.setTurretVoltage(((double)percent) / 100 * 12);
  }
  public void setTurretPosition(double angle) {
    if (!m_turretEnabled) {
      return;
    }
    double min = TurretConstants.BACKWARD_SOFT_LIMIT;
    double max = TurretConstants.FORWARD_SOFT_LIMIT;

    // Wrap angle into range
    while (angle > max) {
        angle -= 360;
    }
    while (angle < min) {
        angle += 360;
    }

    m_io.setTurretPosition(angle);
  }
  public void zeroEncoder () {
    m_io.zeroEncoder();
  }

  public Command setTurretPositionWithController(Turret turret, DoubleSupplier joystickX, DoubleSupplier joystickY, Drive drive) {
    return Commands.run(
      () -> {
        if (!manualControlToggle) {
          return;
        }
        double x = joystickX.getAsDouble();
        double y = joystickY.getAsDouble();
        System.out.println("JstickX: " + x + " JstickY: " + y);
        double angle = Math.toDegrees(Math.atan2(y, x)) + 90;
        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        System.out.println("Angle: " + angle + "\n");
        System.out.println("Magnitude: " + magnitude + "\n");
        if(magnitude > 0.1) {
          setTurretPositionFieldOriented(angle, drive);
        }
      },
      turret
      );
  }

  public void setTurretPositionFieldOriented(double angle, Drive drive) {
    var calculatedAngle = angle - drive.getRawGyroRotation().getDegrees();

    Logger.recordOutput("Turret/FieldOrientedAngle", calculatedAngle);
    setTurretPosition(calculatedAngle);
  }
                                   
  public void setTurretPositionWithCoordinates(Translation2d targetCoordinates, Pose2d robotPose) {
    if (manualControlToggle) {
      return;
    }
    // 1. Get turret position in field coordinates
    Translation2d turretPosFieldRelative = robotPose.getTranslation()
        .plus(Constants.LAUNCHER_POSITION_ROBOT_RELATIVE_M.rotateBy(robotPose.getRotation()));

    // 2. Vector from turret to target
    Translation2d turretToTarget = targetCoordinates.minus(turretPosFieldRelative);

    // 3. Field-relative angle to target
    Rotation2d fieldAngle = turretToTarget.getAngle();

    // 4. Convert to robot-relative
    Rotation2d angleRobotRelative = fieldAngle.minus(robotPose.getRotation());

    setTurretPosition(angleRobotRelative.plus(Rotation2d.k180deg).getDegrees());
  }

  public void targetHub(Drive drive) {
    Pose2d robotPose = drive.getPose();
    setTurretPositionWithCoordinates(Constants.HUB_POSITION_M, robotPose);
  }

  public void toggleManualControl () {
    manualControlToggle = !manualControlToggle;
    Logger.recordOutput("Turret/ManualControlToggled", manualControlToggle);
  }

  public void setBrake(boolean brake) {
    m_io.setBrake(brake);
  }

  /**
   * Returns true if manual control is true
   * @return
   */
  public boolean getManualControlStatus () {
    return manualControlToggle;
  }

  public void autoAim(Drive drive) {
    if (angleStaticSetpoint == 0 || !staticSetpointOverridesTurret) {
      targetHub(drive);
    } else if (angleStaticSetpoint != 0 && staticSetpointOverridesTurret) {
      setTurretPosition(angleStaticSetpoint);
    }
  }

  public void toggleStaticSetpointOverride() {
    staticSetpointOverridesTurret = !staticSetpointOverridesTurret;

    Logger.recordOutput("Turret/StaticSetpointOverridesTurret", staticSetpointOverridesTurret);
  }

  public void setStaticSetpoint(double angle) {
    angleStaticSetpoint = angle;
  }

  public void toggleTurretEnabled () {
    m_turretEnabled = !m_turretEnabled;
    Logger.recordOutput("Turret/Enabled", m_turretEnabled);

    if (!m_turretEnabled) {
      m_io.setTurretVoltage(0);
    }
  }
  // public void targetHub (Pose2d pose) {
    
  // }
}
