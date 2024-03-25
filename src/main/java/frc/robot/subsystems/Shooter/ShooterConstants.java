package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
    public static final int leftCanID = 15;
    public static final int rightCanID = 16;
    
    public static final double kP = 0.2;
    
    public static final double kVTop = 0.118;
    public static final double kVBottom = 0.124;
    
    public static final double RPMTolerance = 150;
    public static final double initialVelocity = 14; // m/s

    public static final InterpolatingDoubleTreeMap shuttleSpeeds = new InterpolatingDoubleTreeMap();

    static {
        shuttleSpeeds.put(10.0, 1500.0);
        shuttleSpeeds.put(17.0, 3000.0);
    }
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
