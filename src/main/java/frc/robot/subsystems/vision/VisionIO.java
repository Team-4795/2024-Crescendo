package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
 
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        Pose3d[] pose = new Pose3d[] {};
        double[] timestamp = new double[] {};

        int[] tags = new int[] {};

        // int goldenBarrelAprilTagDetected = 0;
        // int goldenBarrelNumberOfTags = 0;

        // boolean lifeCamHastargets = false;
        // double lifeCamyaw = 0;

        // private Optional<EstimatedPose> getEstimatedPose(LogTable table, String poseKey, String timestampKey) {
        //     Pose2d newPose = table.get(poseKey, visionPose.map((pose) -> pose.pose()).orElse(null));
        //     Double newTimestamp = table.get(timestampKey, visionPose.map((pose) -> pose.timestamp()).orElse(null));

        //     Optional<EstimatedPose> estPose;

        //     if (newPose != null) {
        //         estPose = Optional.of(new EstimatedPose(newPose, newTimestamp));
        //     } else {
        //         estPose = Optional.empty();
        //     }

        //     return estPose;
        // }

        // @Override
        // public void fromLog(LogTable table) {
        //     visionPose = getEstimatedPose(table, "barbaryFigPose", "barbaryFigPoseTimestamp");
        // }
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}

    public default double getDistanceToTag(int tag) { 
        return 0.0;
    }
} 

