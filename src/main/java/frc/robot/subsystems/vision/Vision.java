package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    PhotonPipelineResult result;
    double yaw;
    double pitch;
    double area;
    double skew;
    double distanceToTarget;
    Transform3d pose;
    List<TargetCorner> corners;
    boolean hasTargets;
    List<PhotonTrackedTarget> targets;
    PhotonTrackedTarget target;

    PhotonCamera Camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d robotToCam;
    PhotonPoseEstimator photonPoseEstimator;
    EstimatedRobotPose visionPose;
    Pose2d speakerPosition;

    public static Vision instance;

    public static Vision getInstance(){
        if(instance == null){
            instance = new Vision();
        }
        return instance;
    }

    private Vision () {
        Camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

        //Cam mounted facing forward, half a meter forward of center, half a meter up from center. Change Later
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 

        try 
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) 
        {
            e.printStackTrace();
        }

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Camera, robotToCam);   

        aprilTagFieldLayout.getTagPose(4).ifPresent(pose -> speakerPosition = pose.toPose2d()); //Get pose2d of speaker
    }

    public Optional<EstimatedRobotPose> getPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public double getDistancetoSpeaker(Pose2d robotPose){
        distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, speakerPosition);
        return distanceToTarget;
    }

    public double getYaw(){
        return yaw;
    }

    public Pose2d getSpeakerPos() {
        return speakerPosition;
    }


    @Override
    public void periodic() {
        result = Camera.getLatestResult();

        hasTargets = result.hasTargets();

        if (hasTargets)
        {
            targets = result.getTargets();
            target = result.getBestTarget();
            yaw = target.getYaw();
            pitch = target.getPitch();
            area = target.getArea();
            skew = target.getSkew();
            pose = target.getBestCameraToTarget();
            corners = target.getDetectedCorners();
        }
    }

}
