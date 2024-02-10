package frc.robot.subsystems.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class PivotConstants {
    public static final int leftCanID = 11;
    public static final int rightCanID = 12;
    public static final double kA = 0;
    public static final double kG = 0;
    public static final double kV = 2.43;
    public static final double kS = 0;
    public static final double kP = 0;
    public static final double KI = 0;
    public static final double kD = 0;

    public static final double angleOffset = Units.degreesToRadians(30);
    public static final double kDt = 0.02;
    public static final Constraints constraints = new Constraints(0.5, 1.5);
    public static final double manualSpeed = 0.01;
    
    public static final double F = 177.929;
    public static final double x = 0.438;
    public static final double y = 0.058;
    public static final double a = 1.58;
    public static final double d = 0.15;
    public static final double M = 9.3;
    public static final double R = 0.393;
    public static final double g = 9.81;

    public static final double gearing = 144;
    public static final double lowLimit = Units.degreesToRadians(30);
    public static final double highLimit = Units.degreesToRadians(105);
}
