// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReadyShoot extends Command {
  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Drive m_drive;
  private final boolean m_passing;
  private boolean m_manualTurretControlEnabled;
  private Translation2d closestPassingTarget;
  /** Creates a new ReadyShoot. */
  public ReadyShoot(Shooter shooter, Turret turret, Drive drive, boolean passing) {
    addRequirements(
      shooter,
      turret
    );

    m_shooter = shooter;
    m_turret = turret;
    m_drive = drive;
    m_passing = passing;
    
    m_manualTurretControlEnabled = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_passing) {
      m_shooter.shoot(m_drive);
      m_turret.autoAim(m_drive);
    } else {
      Pose2d robotPose = m_drive.getPose();
      Translation2d launcherPositionFieldRelative =
        robotPose
          .getTranslation()
          .plus(
            Constants.LAUNCHER_POSITION_ROBOT_RELATIVE_M
            .rotateBy(robotPose.getRotation()));

      closestPassingTarget = Constants.PASSING_TARGETS[0];

      for (int i = 0; i < Constants.PASSING_TARGETS.length; i++) {
        if (launcherPositionFieldRelative.getDistance(Constants.PASSING_TARGETS[i]) < launcherPositionFieldRelative.getDistance(closestPassingTarget)) {
          closestPassingTarget = Constants.PASSING_TARGETS[i];
        }
      }

      var distance = launcherPositionFieldRelative.getDistance(closestPassingTarget);

      m_shooter.passFromDistance(distance);
      
      m_manualTurretControlEnabled = m_turret.getManualControlStatus();
      }
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_passing) {
      m_shooter.shoot(m_drive);
      m_turret.autoAim(m_drive);
    } else {
      Pose2d robotPose = m_drive.getPose();
      Translation2d launcherPositionFieldRelative =
        robotPose
          .getTranslation()
          .plus(
            Constants.LAUNCHER_POSITION_ROBOT_RELATIVE_M
            .rotateBy(robotPose.getRotation()));

      closestPassingTarget = Constants.PASSING_TARGETS[0];

      for (int i = 0; i < Constants.PASSING_TARGETS.length; i++) {
        if (launcherPositionFieldRelative.getDistance(Constants.PASSING_TARGETS[i]) < launcherPositionFieldRelative.getDistance(closestPassingTarget)) {
          closestPassingTarget = Constants.PASSING_TARGETS[i];
        }
      }

      var distance = launcherPositionFieldRelative.getDistance(closestPassingTarget);

      if (m_manualTurretControlEnabled) {
        m_turret.setTurretPositionWithCoordinates(closestPassingTarget, robotPose);
      }

      m_shooter.passFromDistance(distance);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setTurretVoltage(0);
    m_shooter.setShooterVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
