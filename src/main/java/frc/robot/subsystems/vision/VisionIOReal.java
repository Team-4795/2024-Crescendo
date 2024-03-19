package frc.robot.subsystems.vision;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.subsystems.MAXSwerve.Drive;

public class VisionIOReal implements VisionIO {
    PhotonPipelineResult result;

    PhotonCamera camera;

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator poseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

    Pose3d tagPose;

    public VisionIOReal(int camIndex) {
        camera = new PhotonCamera(VisionConstants.cameraIds[camIndex]); 

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.cameraPoses[camIndex]);

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public void setReferencePose(Pose2d reference) {
        poseEstimator.setReferencePose(reference);
    }

    @Override
    public double getDistanceToTag(int tag) {
        aprilTagFieldLayout.getTagPose(tag).ifPresent(pose -> tagPose = pose);
        return PhotonUtils.getDistanceToPose(Drive.getInstance().getPose(), tagPose.toPose2d());
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.visionPose = poseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));

        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            inputs.aprilTagDetected = result.getBestTarget().getFiducialId();
            inputs.numberOfTags = result.getTargets().size();

        }

        // PhotonPipelineResult lifecamResult = LifeCam.getLatestResult();
        // inputs.lifeCamHastargets = lifecamResult.hasTargets();
        // if (inputs.lifeCamHastargets) {
        //     lifecamTarget = lifecamResult.getBestTarget();
        //     inputs.lifeCamyaw = lifecamTarget.getYaw();
        // }
    }
}
