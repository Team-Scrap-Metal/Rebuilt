// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int AUX_CONTROLLER_PORT = 1;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static double MAX_VOLTAGE = 12;

      /** Get Alliance (Blue, Red, Null) */
    public static final Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }

    public static final boolean isAllianceRed() {

      return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? true : false;
    }

    public static final Translation2d LAUNCHER_POSITION_ROBOT_RELATIVE_M 
        = new Translation2d(Units.inchesToMeters(-4.85), Units.inchesToMeters(-8.6)); // center-to-center from center of spindexer shaft -> center of turret gear

    public static final Translation2d HUB_POSITION_M 
        = new Translation2d(4.62534, 4.03479);

}
