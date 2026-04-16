package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    public static final int CAN_ID = 5;
    public static final int FOLLOWER_CAN_ID = 6;
    public static final boolean INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = true;
    public static final int CURRENT_LIMIT = 30;
    public static final double GEAR_RATIO = 2.385185185;
    public static final int SHOOTING_SPEED_RPM = 100;
    public static final double MAX_OUTPUT = 1;
    public static final double MIN_OUTPUT = -1;
    public static final double KP = 0.0005;
    public static final double KI = 0.0;
    public static final double KD = 0.03;
    public static final double KS = 0.185;
    public static final double KV = 0.00211;
    public static final double SHOOTER_ANGLE = 19.11;
    public static final double HUB_HEIGHT_IN = 72.0;
    public static final double SHOOTER_HEIGHT_IN = 18.5;
    public static final double WHEEL_RADIUS_M = 0.1;
    public static final double GRAVITY_CONSTANT = 9.8;

    public static final double SHOOTER_VELOCITY_COEFFICIENT = 1;
    // public static final int SHOOTING_PERCENT = 30;

    public static final double[] BEST_FIT_X_VALUES_M = new double[] {
        Units.inchesToMeters(66),
        Units.inchesToMeters(106.25),
        Units.inchesToMeters(153.5),
        Units.inchesToMeters(202)
    };
    public static final double[] BEST_FIT_RPM_VALUES = new double[] {
        1800,
        2200,
        2400,
        2680
    };

    // TODO: UPDATE PASSING VALUES. FAKE 
    public static final double[] BEST_FIT_PASSING_X_VALUES_M = new double[] {
        Units.inchesToMeters(66),
        Units.inchesToMeters(106.25),
        Units.inchesToMeters(153.5),
        Units.inchesToMeters(202)
    };
    public static final double[] BEST_FIT_PASSING_RPM_VALUES = new double[] {
        1800,
        2200,
        2400,
        2680
    };

    /*** Target RPM to score while flush with hub ***/
    public static final int RPM_FROM_HUB = 1850;
    public static final int RPM_FROM_TRENCH = 2400;

}