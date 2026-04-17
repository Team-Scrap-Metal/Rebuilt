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
    public static final double kMinOutput = -0.8;
    public static final double kMaxOutput = 0.8;
    public static final double POSITION_CONVERSION_FACTOR = 360 / GEAR_RATIO;

    public static final boolean FORWARD_SOFT_LIMIT_ENABLED = true;
    public static final boolean BACKWARD_SOFT_LIMIT_ENABLED = true;
    public static final double FORWARD_SOFT_LIMIT = -4;
    public static final double BACKWARD_SOFT_LIMIT = -358;
    public static final boolean TURRET_DEFAULT_MANUAL_CONTROL = false;
    
    public static final double STARTING_ANGLE = -90;

    public static final double ANGLE_FOR_STATIC_HUB = -90;
    public static final double ANGLE_FOR_STATIC_TRENCH_LEFT = -280;
    public static final double ANGLE_FOR_STATIC_TRENCH_RIGHT = -100;

    public static final boolean TURRET_ENABLED_DEFAULT = true;
}