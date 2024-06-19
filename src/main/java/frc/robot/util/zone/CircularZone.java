package frc.robot.util.zone;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CircularZone extends Trigger{
    private double radius;
    private Translation2d center;
    public CircularZone(double radius, Translation2d center, Supplier<Pose2d> pose) {
        super(() -> {
            return center.getDistance(pose.get().getTranslation()) <= radius;
        });
        this.center = center;
        this.radius = radius;
    }
    public double getRadius() {
        return radius;
    }
    public Translation2d getCenter() {
        return center;
    }
}
