package frc.robot.subsystems.pivot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class PivotConstants {
    public static final int leftCanID = 11;
    public static final int rightCanID = 12;
    public static final double kA = 0.14;
    public static final double kV = 2.35;
    public static final double kS = 0.15;
    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double angleOffset = 0.24;
    public static final double kDt = 0.02;
    public static final Constraints constraints = new Constraints(4.5, 7); //rad/s and rad/s^2
    public static final double manualSpeed = 0.01;
    
    public static final double F = 178;
    public static final double x = 0.438;
    public static final double y = 0.038;
    public static final double d = 0.152;
    public static final double M = 10.0;
    public static final double R = 0.393;
    public static final double g = -9.81;
    public static final double inertia = 1.9;

    public static final double positionConversionFactor = Math.PI * 2.0;
    public static final double velocityConversionFactor = Math.PI * 2.0;

    public static final double gearing = 144;
    public static final double lowLimit = Units.degreesToRadians(3);
    public static final double highLimit = Units.degreesToRadians(80);
    public static final double height = 0.2794;
    public static final double offset = 0.25;

    public static final InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();

    static {
        armAngleMap.put(1.3, 0.6);
        armAngleMap.put(1.7, 0.427);
        armAngleMap.put(2.6, 0.28);
        armAngleMap.put(3.75, 0.185);
        armAngleMap.put(4.6, 0.135);
        armAngleMap.put(4.9, 0.13);
        armAngleMap.put(5.7, 0.112);
        armAngleMap.put(6.1, 0.107);
        armAngleMap.put(6.4, 0.095);
    }
}
