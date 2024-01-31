package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15.0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
}
