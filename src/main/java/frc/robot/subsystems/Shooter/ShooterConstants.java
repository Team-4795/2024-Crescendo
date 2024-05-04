package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
    public static final int leftCanID = 15;
    public static final int rightCanID = 16;
    
    public static final double kP = 0.15;
    
    public static final double kVTop = 0.116;
    public static final double kVBottom = 0.121;
    
    public static final double initialVelocity = 14; // m/s

    public static final InterpolatingDoubleTreeMap shuttleSpeeds = new InterpolatingDoubleTreeMap();

    static {
        shuttleSpeeds.put(5.0, 2400.0);
        shuttleSpeeds.put(9.5, 3000.0);
    }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
