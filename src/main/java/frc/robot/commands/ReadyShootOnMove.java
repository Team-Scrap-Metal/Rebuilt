package frc.robot.commands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConstants;
import frc.robot.Subsystems.Turret.Turret;

public class ReadyShootOnMove extends Command {
  private final double TIME_OF_FLIGHT_SCALAR = 1;

  private final LoggedNetworkNumber tofScalarInput;

  private final Drive m_drive;
  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Translation2d m_target;

  public ReadyShootOnMove(
      Translation2d target,
      Drive drive,
      Shooter shooter,
      Turret turret
  ) {
    m_target = target;
    m_drive = drive;
    m_shooter = shooter;
    m_turret = turret;

    tofScalarInput = new LoggedNetworkNumber("Tuning/TOF-Scalar", TIME_OF_FLIGHT_SCALAR);

    addRequirements(m_shooter, m_turret);
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drive.getPose();

    // --- launcher position ---
    Translation2d launcherPos = m_shooter.getLauncherPose(robotPose);

    // --- base geometry ---
    Translation2d toTarget = m_target.minus(launcherPos);
    double distance = toTarget.getNorm();

    double baseRPM = m_shooter.calculateRpm(distance);

    // RPM /velocity 
    double linearVelocity =
        (baseRPM * 2.0 * Math.PI / 60.0)
        * ShooterConstants.WHEEL_RADIUS_M;

    double angleRad = Math.toRadians(ShooterConstants.SHOOTER_ANGLE);

    double heightDifference =
        Units.inchesToMeters(ShooterConstants.HUB_HEIGHT_IN) -
        Units.inchesToMeters(ShooterConstants.SHOOTER_HEIGHT_IN);

    // time of flight
    double g = 9.81;
    double vy = linearVelocity * Math.sin(angleRad);

    double a = 0.5 * g;
    double b = -vy;
    double c = heightDifference;

    double discriminant = b * b - 4 * a * c;

    double time;

    if (discriminant < 0) {
      time = 0.0;
    } else {
      double sqrt = Math.sqrt(discriminant);
      double t1 = (-b + sqrt) / (2 * a);
      double t2 = (-b - sqrt) / (2 * a);
      time = Math.max(t1, t2);
    }

    // scalar tuning-
    time *= tofScalarInput.get();

    ChassisSpeeds speeds = m_drive.getChassisSpeeds();

    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            speeds,
            robotPose.getRotation()
        );

    Translation2d robotVel = new Translation2d(
        fieldRelative.vxMetersPerSecond,
        fieldRelative.vyMetersPerSecond
    );

    Translation2d compensatedTarget =
        m_target.minus(robotVel.times(time));

    m_turret.setTurretPositionWithCoordinates(
        compensatedTarget,
        robotPose
    );

    double compensatedDistance =
        launcherPos.getDistance(compensatedTarget);

    m_shooter.revForDistance(compensatedDistance);

    Logger.recordOutput("Shooter/BaseDistance", distance);
    Logger.recordOutput("Shooter/CompensatedDistance", compensatedDistance);
    Logger.recordOutput("Shooter/TimeOfFlight", time);
    Logger.recordOutput("Shooter/VelocityX", robotVel.getX());
    Logger.recordOutput("Shooter/VelocityY", robotVel.getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}