package frc.robot.Subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class TurretConstants {
    public static final boolean BRAKE_MODE_COAST = true;
    public static final int CAN_ID = 12;
    public static final boolean INVERTED = true;
    public static final int CURRENT_LIMIT = 20;
    public static final double GEAR_RATIO = 100;
    public static final double kP = 0.02;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.2;
    public static final double kV = 0.2;
    public static final double kA = 0;
    public static final double kMinOutput = 0;
    public static final double kMaxOutput = 0;
    public static final double POSITION_CONVERSION_FACTOR = 360 / GEAR_RATIO;
    public static final boolean FORWARD_SOFT_LIMIT_ENABLED = true;
    public static final boolean BACKWARD_SOFT_LIMIT_ENABLED = true;
    public static final double FORWARD_SOFT_LIMIT = 86;
    public static final double BACKWARD_SOFT_LIMIT = -60;
    public static final boolean TURRET_DEFAULT_MANUAL_CONTROL = false;
    
    public static final Translation2d TURRET_POSITION_ROBOT_RELATIVE_M 
        = new Translation2d(Units.inchesToMeters(-4.85), Units.inchesToMeters(-8.6)); // center-to-center from center of spindexer shaft -> center of turret

    public static final Translation2d HUB_POSITION_M 
        = new Translation2d(4.62534, 4.03479); // TODO: update
}