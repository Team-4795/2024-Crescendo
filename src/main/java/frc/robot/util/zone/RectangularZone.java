package frc.robot.util.zone;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RectangularZone extends Trigger {
    private double x1;
    private double x2;
    private double y1;
    private double y2;
    public RectangularZone(double x1, double y1, double x2, double y2, Supplier<Pose2d> pose) {
        super(
            () ->   
                pose.get().getX() >= x1 &&
                pose.get().getY() >= y1 &&
                pose.get().getX() <= x2 &&
                pose.get().getY() <= y2
        );
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
    }

    public Translation2d getCorner1() {
        return new Translation2d(x1, y1);
    }

    public Translation2d getCorner2() {
        return new Translation2d(x2, y2);
    }
}
