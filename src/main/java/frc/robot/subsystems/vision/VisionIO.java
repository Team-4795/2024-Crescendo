package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
 
public interface VisionIO {
    public record EstimatedPose(Pose2d pose, double timestamp) {}

    public static class VisionIOInputs {
        Optional<EstimatedPose> barbaryFigPose = Optional.empty();
        Optional<EstimatedPose> saguaroPose = Optional.empty();
        Optional<EstimatedPose> goldenBarrelPose = Optional.empty();

        int barbaryFigAprilTagDetected = 0;
        int barbaryFigNumberOfTags = 0;

        int saguaroAprilTagDetected = 0;
        int saguaroNumberOfTags = 0;

        int goldenBarrelAprilTagDetected = 0;
        int goldenBarrelNumberOfTags = 0;

        boolean lifeCamHastargets = false;
        double lifeCamyaw = 0;
    }

    public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
        @Override
        public void toLog(LogTable table) {
            table.put("barbaryFigPose", barbaryFigPose.map((pose) -> pose.pose()).orElse(null));
            barbaryFigPose.ifPresent((pose) -> table.put("barbaryFigTimestamp", pose.timestamp()));

            table.put("saguaroPose", saguaroPose.map((pose) -> pose.pose()).orElse(null));
            saguaroPose.ifPresent((pose) -> table.put("saguaroFigTimestamp", pose.timestamp()));

            table.put("goldenBarrelPose", goldenBarrelPose.map((pose) -> pose.pose()).orElse(null));
            goldenBarrelPose.ifPresent((pose) -> table.put("goldenBarrelFigTimestamp", pose.timestamp()));
        }

        private Optional<EstimatedPose> getEstimatedPose(LogTable table, String poseKey, String timestampKey) {
            Pose2d newPose = table.get(poseKey, barbaryFigPose.map((pose) -> pose.pose()).orElse(null));
            Double newTimestamp = table.get(timestampKey, barbaryFigPose.map((pose) -> pose.timestamp()).orElse(null));

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
            barbaryFigPose = getEstimatedPose(table, "barbaryFigPose", "barbaryFigPoseTimestamp");
            saguaroPose = getEstimatedPose(table, "saguaroPose", "saguaroPoseTimestamp");
            goldenBarrelPose = getEstimatedPose(table, "goldenBarrelPose", "saguaroPoseTimestamp");
        }

        @Override
        public VisionIOInputsAutoLogged clone() {
            VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
            copy.barbaryFigPose = this.barbaryFigPose;
            copy.saguaroPose = this.saguaroPose;
            copy.goldenBarrelPose = this.goldenBarrelPose;
            return copy;
        }
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}

    public default double getDistanceToTag(int tag) { 
        return 0.0;
    }
} 

