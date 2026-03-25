package frc.robot.subsystems.Gyro;

public final class GyroConstants {
    public static final int CAN_ID = 9; // TODO: update
    public static final int FOLLOWER_CAN_ID = 16; // TODO: update
    public static final boolean INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = true;
    public static final int CURRENT_LIMIT = 30;
    public static final double GEAR_RATIO = 1;
    public static final int INTAKING_gyro_PERCENT = 50;
    public static final int LAUNCHING_gyro_PERCENT = 80;

    public static enum GyroState {
        INTAKING,
        LAUNCHING
    }

    public static GyroState currentState = GyroState.INTAKING;
}