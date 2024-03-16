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
    PhotonPipelineResult saguaroResult;
    PhotonPipelineResult barbaryFigResult;

    PhotonCamera SaguaroCam;
    PhotonCamera BarbaryFig;
    PhotonCamera goldenBarrel;

    PhotonCamera LifeCam;

    PhotonTrackedTarget saguaroTarget;
    PhotonTrackedTarget barbaryFigTarget;
    PhotonTrackedTarget goldenBarrelTarget;
    PhotonTrackedTarget lifecamTarget;

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d saguaroRobotToCam;
    Transform3d barbaryFigRobotToCam;
    Transform3d goldenBarrelRobotToCam;
    PhotonPoseEstimator saguaroPhotonPoseEstimator;
    PhotonPoseEstimator barbaryFigPhotonPoseEstimator;
    PhotonPoseEstimator goldenBarrelPhotonPoseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

    Pose3d tagPose;

    public VisionIOReal() {
        SaguaroCam = new PhotonCamera("Saguaro");
        BarbaryFig = new PhotonCamera("Barbary Fig");
        goldenBarrel = new PhotonCamera("Golden Barrel");
        LifeCam = new PhotonCamera("Queen of the Night");
        

        saguaroRobotToCam = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-10), 
                Units.inchesToMeters(8.5), 
                Units.inchesToMeters(9)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Units.degreesToRadians(110)));

        barbaryFigRobotToCam = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.5),
                Units.inchesToMeters(5), 
                Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Math.PI));

        goldenBarrelRobotToCam = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-10),
                Units.inchesToMeters(-8.5), 
                Units.inchesToMeters(9)),
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Units.degreesToRadians(-110)));  

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        saguaroPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, SaguaroCam, saguaroRobotToCam);
                    
        barbaryFigPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BarbaryFig, barbaryFigRobotToCam);

        goldenBarrelPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, goldenBarrel, goldenBarrelRobotToCam);

        saguaroPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        barbaryFigPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        goldenBarrelPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public void setReferencePose(Pose2d reference) {
        barbaryFigPhotonPoseEstimator.setReferencePose(reference);
        saguaroPhotonPoseEstimator.setReferencePose(reference);
        goldenBarrelPhotonPoseEstimator.setReferencePose(reference);
    }

    @Override
    public double getDistanceToTag(int tag) {
        aprilTagFieldLayout.getTagPose(tag).ifPresent(pose -> tagPose = pose);
        return PhotonUtils.getDistanceToPose(Drive.getInstance().getPose(), tagPose.toPose2d());
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.barbaryFigPose = barbaryFigPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
        inputs.saguaroPose = saguaroPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
        inputs.goldenBarrelPose = goldenBarrelPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));

        PhotonPipelineResult barbaryFigResult = BarbaryFig.getLatestResult();
        if (barbaryFigResult.hasTargets()) {
            inputs.barbaryFigAprilTagDetected = barbaryFigResult.getBestTarget().getFiducialId();
            inputs.barbaryFigNumberOfTags = barbaryFigResult.getTargets().size();
        }

        PhotonPipelineResult saguaroResult = SaguaroCam.getLatestResult();
        if (saguaroResult.hasTargets()) {
            inputs.saguaroAprilTagDetected = saguaroResult.getBestTarget().getFiducialId();
            inputs.saguaroNumberOfTags = saguaroResult.getTargets().size();
        }

        PhotonPipelineResult goldenBarrelResult = goldenBarrel.getLatestResult();
        if (goldenBarrelResult.hasTargets()) {
            inputs.goldenBarrelAprilTagDetected = goldenBarrelResult.getBestTarget().getFiducialId();
            inputs.goldenBarrelNumberOfTags = goldenBarrelResult.getTargets().size();
        }

        PhotonPipelineResult lifecamResult = LifeCam.getLatestResult();
        inputs.lifeCamHastargets = lifecamResult.hasTargets();
        if (inputs.lifeCamHastargets) {
            lifecamTarget = lifecamResult.getBestTarget();
            inputs.lifeCamyaw = lifecamTarget.getYaw();
        }
    }
}
