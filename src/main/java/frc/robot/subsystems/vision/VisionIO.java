package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
 
public interface VisionIO {
    public record EstimatedPose(Pose2d pose, double timestamp) {}

    public static class VisionIOInputs {
        Optional<EstimatedPose> visionPose = Optional.empty();

        int aprilTagDetected = 0;
        int numberOfTags = 0;

        int goldenBarrelAprilTagDetected = 0;
        int goldenBarrelNumberOfTags = 0;

        boolean lifeCamHastargets = false;
        double lifeCamyaw = 0;
    }

    public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
        @Override
        public void toLog(LogTable table) {
            table.put("barbaryFigPose", visionPose.map((pose) -> pose.pose()).orElse(null));
            visionPose.ifPresent((pose) -> table.put("barbaryFigTimestamp", pose.timestamp()));

        }

        private Optional<EstimatedPose> getEstimatedPose(LogTable table, String poseKey, String timestampKey) {
            Pose2d newPose = table.get(poseKey, visionPose.map((pose) -> pose.pose()).orElse(null));
            Double newTimestamp = table.get(timestampKey, visionPose.map((pose) -> pose.timestamp()).orElse(null));

            Optional<EstimatedPose> estPose;

            if (newPose != null) {
                estPose = Optional.of(new EstimatedPose(newPose, newTimestamp));
            } else {
                estPose = Optional.empty();
            }

            return estPose;
        }

        @Override
        public void fromLog(LogTable table) {
            visionPose = getEstimatedPose(table, "barbaryFigPose", "barbaryFigPoseTimestamp");
        }

        @Override
        public VisionIOInputsAutoLogged clone() {
            VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
            copy.visionPose = this.visionPose;
            return copy;
        }
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}

    public default double getDistanceToTag(int tag) { 
        return 0.0;
    }
} 

