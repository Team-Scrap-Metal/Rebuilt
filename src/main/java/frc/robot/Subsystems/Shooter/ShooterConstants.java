package frc.robot.Subsystems.Shooter;

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
    public static final double KP = 0.0002;
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

    /*** Target RPM to score while flush with hub ***/
    public static final int RPM_FROM_HUB = 500;
}