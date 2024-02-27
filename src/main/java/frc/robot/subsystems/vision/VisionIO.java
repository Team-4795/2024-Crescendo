package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
 
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public double saguaroRoll = 0.0;
        public double saguaroPitch = 0.0;
        public double saguaroYaw = 0.0;

        public double barbaryFigRoll = 0.0;
        public double barbaryFigPitch = 0.0;
        public double barbaryFigYaw = 0.0;

        public Optional<EstimatedRobotPose> saguaroPose;
        public Optional<EstimatedRobotPose> barbaryFigPose;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default Pose2d getSpeakerPos() {
        return null;
    }

    public default PhotonPoseEstimator getBarbaryFigPoseEstimator(){
        return null;
    }

    public default PhotonPoseEstimator getSaguaroPoseEstimator(){
        return null;
    }

} 

