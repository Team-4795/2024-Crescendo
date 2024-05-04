package frc.robot.subsystems.vision.AprilTagVision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOReal implements VisionIO {
    PhotonPipelineResult result;

    PhotonCamera camera;

    PhotonPoseEstimator poseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

    Pose3d tagPose;

    public VisionIOReal(int camIndex) {
        camera = new PhotonCamera(VisionConstants.cameraIds[camIndex]); 

        poseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.cameraPoses[camIndex]);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void setReferencePose(Pose2d reference) {
        poseEstimator.setReferencePose(reference);
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        poseEstimator.update().ifPresentOrElse((pose) -> {
            inputs.pose = new Pose3d[] {pose.estimatedPose};
            inputs.timestamp = new double[] {pose.timestampSeconds};
            inputs.tags = pose.targetsUsed.stream().mapToInt((target) -> target.getFiducialId()).toArray();
        }, () -> {
            inputs.pose = new Pose3d[] {};
            inputs.timestamp = new double[] {};
            inputs.tags = new int[] {};
        });
    }
}