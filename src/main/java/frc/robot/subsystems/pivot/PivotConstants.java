package frc.robot.subsystems.pivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class PivotConstants {
    public static final int leftCanID = 12;
    public static final int rightCanID = 13;

    public static final double kP = 0; 
    public static final double kI = 0; 
    public static final double kD = 0;

    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final Constraints constraints = new Constraints(2, 2);
}
