// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(25.750);
  public static final double wheelBase = Units.inchesToMeters(17.250); //14
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };
  
  public static final double BUMPER_THICKNESS_IN = 2;
  /** bumper to bumper robot width */
  public static final double ROBOT_WIDTH_BTB = Units.inchesToMeters(31.500 + BUMPER_THICKNESS_IN*2);
  public static final double ROBOT_LENGTH_BTB = Units.inchesToMeters(23.125 + BUMPER_THICKNESS_IN*2);

  
  public static final boolean[] driveInverted = new boolean[] {
    false,
    false,
    false,
    false
  };

  // Zeroed rotation values for each module, see setup instructions
  // TODO: zero module values
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-2.523);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.663);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(1.525);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.319);
  // public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.728);
  // public static final Rotation2d frontRightZeroRotation = new Rotation2d(-2.489);
  // public static final Rotation2d backLeftZeroRotation = new Rotation2d(-1.594);
  // public static final Rotation2d backRightZeroRotation = new Rotation2d(-2.815);

  //TODO: can IDs
  // Device CAN IDs
  public static final int pigeonCanId = 23;

  public static final int frontLeftDriveCanId = 4;
  public static final int backLeftDriveCanId = 2;
  public static final int frontRightDriveCanId = 15;
  public static final int backRightDriveCanId = 13;

  public static final int frontLeftTurnCanId = 18;
  public static final int backLeftTurnCanId = 21;
  public static final int frontRightTurnCanId = 10;
  public static final int backRightTurnCanId = 14;

  
  public static final int frontLeftAbsoluteCanId = 24;
  public static final int backLeftAbsoluteCanId = 22;
  public static final int frontRightAbsoluteCanId = 35;
  public static final int backRightAbsoluteCanId = 33;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 40;
  public static final int driveMotorSlipCurrent = 80;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2); 
  public static final double driveMotorReduction = 6.12;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double[] driveKp = new double[] {
      0.005,
      0.0,
      0.0,
      0.02
  };
  public static final double[] driveKd = new double[] {
    0.0,
    0.0,
    0.0,
    0.0
  };
  public static final double[] driveKs = new double[] {
    0.179,
    0.179,
    0.179,
    0.179
  };
  public static final double[] driveKv = new double[] {
    0.12,
    0.12,
    0.12,
    0.21
  }; 
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 18.75 / 1;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double[] turnKp = new double[] {
    5.78,
    5.95,
    5.95,
    5.95
  };
  public static final double[] turnKd = new double[] {
    0.12,
    0.1,
    0.1,
    0.1
  };
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = -Math.PI; // Radians
  public static final double turnPIDMaxInput = Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final double BUMPER_WIDTH_M = Units.inchesToMeters(2);
}
