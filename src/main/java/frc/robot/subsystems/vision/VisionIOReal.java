package frc.robot.subsystems.vision;

import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.MAXSwerve.Drive;

public class VisionIOReal implements VisionIO {
    // PhotonPipelineResult saguaroResult;
    PhotonPipelineResult barbaryFigResult;

    // PhotonCamera SaguaroCam;
    PhotonCamera BarbaryFig;
    PhotonCamera LifeCam;

    // PhotonTrackedTarget saguaroTarget;
    PhotonTrackedTarget barbaryFigTarget;
    PhotonTrackedTarget lifecamTarget;

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d saguaroRobotToCam;
    Transform3d barbaryFigRobotToCam;
    PhotonPoseEstimator saguaroPhotonPoseEstimator;
    PhotonPoseEstimator barbaryFigPhotonPoseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

    Pose3d tagPose;

    public VisionIOReal() {
        // SaguaroCam = new PhotonCamera("Saguaro");
        BarbaryFig = new PhotonCamera("Barbary Fig");
        LifeCam = new PhotonCamera("Queen of the Night");
        

        saguaroRobotToCam = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-8), 
                Units.inchesToMeters(-6.5), 
                Units.inchesToMeters(10.5)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Units.degreesToRadians(-110)));

        barbaryFigRobotToCam = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-20), 
                Units.inchesToMeters(5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                Math.PI, 
                Units.degreesToRadians(20), 
                Math.PI));

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // saguaroPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        //              PoseStrategy.CLOSEST_TO_REFERENCE_POSE, SaguaroCam, saguaroRobotToCam);
                    
        barbaryFigPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, BarbaryFig, barbaryFigRobotToCam);

        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public void setReferencePose(Pose2d reference) {
        barbaryFigPhotonPoseEstimator.setReferencePose(reference);
    }

    @Override
    public double getDistanceToTag(int tag) {
        aprilTagFieldLayout.getTagPose(tag).ifPresent(pose -> tagPose = pose);
        return PhotonUtils.getDistanceToPose(Drive.getInstance().getPose(), tagPose.toPose2d());
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.barbaryFigPose = barbaryFigPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
        inputs.saguaroPose = barbaryFigPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));

        PhotonPipelineResult barbaryFigResult = BarbaryFig.getLatestResult();
        if (barbaryFigResult.hasTargets()) {
            inputs.barbaryFigAprilTagDetected = barbaryFigResult.getBestTarget().getFiducialId();
            inputs.barbaryFigNumberOfTags = barbaryFigResult.getTargets().size();
        }

        // PhotonPipelineResult saguaroResult = SaguaroCam.getLatestResult();
        // if (saguaroResult.hasTargets()) {
        //     inputs.saguaroAprilTagDetected = saguaroResult.getBestTarget().getFiducialId();
        //     inputs.saguaroNumberOfTags = saguaroResult.getTargets().size();
        // }

        PhotonPipelineResult lifecamResult = LifeCam.getLatestResult();
        inputs.lifeCamHastargets = lifecamResult.hasTargets();
        if (inputs.lifeCamHastargets) {
            lifecamTarget = lifecamResult.getBestTarget();
            inputs.lifeCamyaw = lifecamTarget.getYaw();
        }
    }
}
