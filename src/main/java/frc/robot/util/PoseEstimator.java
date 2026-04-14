package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// import java.util.logging.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveConstants;
import frc.robot.Subsystems.Vision.Vision.VisionConsumer;

/** This class handels the odometry and locates the robots current position */
public class PoseEstimator extends SubsystemBase {
  /**
   * Increase the numbers to trust the model's state estimate less it is a matrix in form of [x, y,
   * theta] or meters, meters, radians
   */
  public static Vector<N3> stateStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  public static Vector<N3> visionStandardDevs = VecBuilder.fill(0.1, 0.1, 9999999);

  private boolean visionResetsOdometry = true;

  private SwerveDrivePoseEstimator poseEstimator;
  private Drive drive;
  private Field2d field2d;

  private LoggedDashboardChooser<Pose2d> m_startingPoseChooser;
  private Pose2d m_oldPoseChooserValue;
//   private LimelightHelpers.PoseEstimate mt1;

  public PoseEstimator(Drive drive) {
    m_startingPoseChooser.addOption(
      "Left", 
      new Pose2d(
        Units.inchesToMeters(158.6 - 7), // frame perimeter flush with starting line. Robot rotation zero'd off edge of field
        Units.inchesToMeters(317.7 - 11.125), // flush with field wall
        Rotation2d.k180deg) // Intake facing driver station
      );
    m_startingPoseChooser.addDefaultOption(
      "Center", 
      new Pose2d(
        Units.inchesToMeters(158.6 - 7) - DriveConstants.BUMPER_WIDTH_M, // bumpers flush with hub
        Units.inchesToMeters(317.7/2), // center of field
        Rotation2d.fromDegrees(-90)) // left robot side against hub
      );
    m_startingPoseChooser.addOption(
      "Right", 
      new Pose2d(
        Units.inchesToMeters(158.6 - 7), // frame perimeter flush with starting line. Robot rotation zero'd off edge of field
        Units.inchesToMeters(11.125), // flush with field wall
        Rotation2d.k180deg) // Intake facing driver station
      );
    
    m_oldPoseChooserValue = m_startingPoseChooser.get();
    
    field2d = new Field2d();
    field2d.setRobotPose(m_oldPoseChooserValue);

    SmartDashboard.putData(field2d);
    this.drive = drive;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.moduleTranslations),
            drive.getRawGyroRotation(),
            drive.getModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()),
            stateStandardDevs,
            visionStandardDevs);

    // mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
  }

  @Override
  public void periodic() {
    // When ran on the real robot it would overload the command scheduler, causing input delay from
    // joystick to driving
    field2d.setRobotPose(getPose());

    if (m_oldPoseChooserValue != m_startingPoseChooser.get()) {
      field2d.setRobotPose(m_startingPoseChooser.get());
      poseEstimator.resetPose(m_startingPoseChooser.get());
    }

    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), drive.getRawGyroRotation(), drive.getModulePositions());

    // System.out.println(mt1.tagCount);
    // System.out.println(mt1.pose);

    // mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // if (mt1.tagCount > 0) {
    //   // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0, 0, 0));
    //   poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
    //   // System.out.println("running");
    // }
  }

  /**
   * @return the current pose in a Pose2d
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  /**
   * Resets the pose
   *
   * @param currentPose2d
   */
  public void resetPose(Pose2d currentPose2d) {
    poseEstimator.resetPosition(drive.getRawGyroRotation(), drive.getModulePositions(), currentPose2d);
  }
  /**
   * @return the rotation in a Rotation2d in degrees
   */
  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
      Logger.recordOutput("Vision/VisionPose", visionRobotPoseMeters);
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    
    if (visionResetsOdometry && visionRobotPoseMeters != null) {
      resetPose(visionRobotPoseMeters);
    }
  }
}