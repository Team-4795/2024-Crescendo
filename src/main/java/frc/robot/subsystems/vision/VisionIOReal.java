package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class VisionIOReal implements VisionIO {
    PhotonPipelineResult saguaroResult;
    PhotonPipelineResult barbaryFigResult;

    PhotonCamera SaguaroCam;
    PhotonCamera BarbaryFig;

    PhotonTrackedTarget saguaroTarget;
    PhotonTrackedTarget barbaryFigTarget;

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d saguaroRobotToCam;
    Transform3d barbaryFigRobotToCam;
    PhotonPoseEstimator saguaroPhotonPoseEstimator;
    PhotonPoseEstimator barbaryFigPhotonPoseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

  public VisionIOReal() {
    SaguaroCam = new PhotonCamera("Saguaro");
    BarbaryFig = new PhotonCamera("Barbary Fig");

    saguaroRobotToCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-8), 
            Units.inchesToMeters(-6.5), 
            Units.inchesToMeters(10.5)), 
        new Rotation3d(
            0, 
            Units.degreesToRadians(20), 
            Units.degreesToRadians(-90)));

    barbaryFigRobotToCam = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-10.5), 
            Units.inchesToMeters(5), 
            Units.inchesToMeters(11)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(20), 
                Math.PI));

    try {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
        e.printStackTrace();
    }

    saguaroPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, SaguaroCam, saguaroRobotToCam);
                
    barbaryFigPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, BarbaryFig, barbaryFigRobotToCam);

    aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    getSpeakerPos();
}
    @Override
    public Optional<EstimatedRobotPose> getSaguaroPose(Pose2d reference){
        saguaroPhotonPoseEstimator.setReferencePose(reference);
        return saguaroPhotonPoseEstimator.update();
    }

    @Override
    public Optional<EstimatedRobotPose> getBarbaryFigPose(Pose2d reference){
        barbaryFigPhotonPoseEstimator.setReferencePose(reference);
        return barbaryFigPhotonPoseEstimator.update();
    }

    @Override
    public Pose2d getSpeakerPos() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                aprilTagFieldLayout.getTagPose(4).ifPresent(pose -> speakerPosition = pose.toPose2d());
            } else if (ally.get() == Alliance.Blue) {
                aprilTagFieldLayout.getTagPose(7).ifPresent(pose -> speakerPosition = pose.toPose2d());
            }
        } else {
            speakerPosition = new Pose2d();
        }
        return speakerPosition;
    }


    @Override
    public void updateInputs(VisionIOInputs inputs) {
        PhotonPipelineResult saguaroResult = SaguaroCam.getLatestResult();
        PhotonPipelineResult barbaryFigResult = BarbaryFig.getLatestResult();

        boolean saguaroHasTargets = saguaroResult.hasTargets();
        boolean barbaryFigHasTargets = barbaryFigResult.hasTargets();

        if (saguaroHasTargets)
        {
            saguaroTarget = saguaroResult.getBestTarget();

            inputs.saguaroRoll = saguaroTarget.getSkew();
            inputs.saguaroPitch = saguaroTarget.getPitch();
            inputs.saguaroYaw = saguaroTarget.getYaw();
        }

        if (barbaryFigHasTargets)
        {
            barbaryFigTarget = barbaryFigResult.getBestTarget();

            inputs.barbaryFigRoll = barbaryFigTarget.getSkew();
            inputs.barbaryFigPitch = barbaryFigTarget.getPitch();
            inputs.barbaryFigYaw = barbaryFigTarget.getYaw();
        }

    }
}
