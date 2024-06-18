package frc.robot.util.zone;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CircularZone extends Trigger{
    private double radius;
    private double x;
    private double y;
    public CircularZone(double radius, double centerX, double centerY, Supplier<Pose2d> pose) {
        super(() -> {
            return Math.sqrt(Math.pow(centerX - pose.get().getX(),2) + Math.pow(centerY - pose.get().getY(),2)) <= radius;
        });
        this.x = centerX;
        this.y = centerY;
        this.radius = radius;
    }
    public double getRadius() {
        return radius;
    }
    public Translation2d getCenter() {
        return new Translation2d(x, y);
    }
}
