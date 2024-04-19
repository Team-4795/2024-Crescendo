package frc.robot.subsystems.vision.AprilTagVision;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.5;

    public static final double xyStdDevSingleTag = 0.03;
    public static final double xyStdDevMultiTag = 0.018;

    public static final String[] cameraIds =
    new String[] {
        "Barbary Fig",
        "Saguaro",
        "Golden Barrel"
      };

    public static final Transform3d[] cameraPoses =
    new Transform3d[] {
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-10.5),
                Units.inchesToMeters(5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(-20), 
                Math.PI)),
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-8),
                Units.inchesToMeters(-7.5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(-20), 
                Units.degreesToRadians(-110))),
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-8), 
                Units.inchesToMeters(7.5), 
                Units.inchesToMeters(10)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(-20), 
                Units.degreesToRadians(110)))
    };

    public static AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static final double areaCutoff = 11;
    public static final double timeDelay = 0.5;
    public static final double intakeCamOffset = 0.3;
    public static final double degreeTolerance = 6.5;
}
