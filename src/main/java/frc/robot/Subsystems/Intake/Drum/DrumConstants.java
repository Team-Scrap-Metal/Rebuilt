package frc.robot.Subsystems.Intake.Drum;

public final class DrumConstants {
    public static final int CAN_ID = 9; // TODO: update
    public static final int FOLLOWER_CAN_ID = 16; // TODO: update
    public static final boolean INVERTED = true;
    public static final boolean FOLLOWER_INVERTED = true;
    public static final int CURRENT_LIMIT = 30;
    public static final double GEAR_RATIO = 1;
    public static final int INTAKING_DRUM_PERCENT = 30;
    public static final int LAUNCHING_DRUM_PERCENT = 100;

    public static enum DrumState {
        INTAKING,
        LAUNCHING
    }

    public static DrumState currentState = DrumState.INTAKING;
}