package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.opencv.aruco.EstimateParameters;
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

    // PhotonCamera SaguaroCam;
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
        // SaguaroCam = new PhotonCamera("Saguaro");
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
                Units.inchesToMeters(-11), 
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

        // saguaroPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        //             PoseStrategy.CLOSEST_TO_REFERENCE_POSE, SaguaroCam, saguaroRobotToCam);
                    
        barbaryFigPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, BarbaryFig, barbaryFigRobotToCam);

        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    @Override
    public void setReferencePose(Pose2d reference) {
        barbaryFigPhotonPoseEstimator.setReferencePose(reference);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.barbaryFigPose = barbaryFigPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
        inputs.saguaroPose = barbaryFigPhotonPoseEstimator.update().map((pose) -> new EstimatedPose(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
    }
}
