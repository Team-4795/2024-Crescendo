package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.littletonrobotics.junction.Logger;
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
    PhotonPipelineResult arducamResult;
    PhotonPipelineResult lifecamResult;
    double arducamYaw;
    double lifecamYaw;
    double arducamPitch;
    double lifecamPitch;
    double arducamArea;
    double lifecamArea;
    double arducamSkew;
    double lifecamSkew;
    double distanceToTarget;
    Transform3d arducamPose;
    Transform3d lifecamPose;
    List<TargetCorner> arducamCorners;
    List<TargetCorner> lifecamCorners;
    boolean arducamHasTargets;
    boolean lifecamHasTargets;
    List<PhotonTrackedTarget> arducamTargets;
    List<PhotonTrackedTarget> lifecamTargets;
    PhotonTrackedTarget arducamTarget;
    PhotonTrackedTarget lifecamTarget;

    PhotonCamera Arducam;
    PhotonCamera Lifecam;

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d arducamRobotToCam;
    Transform3d lifecamRobotToCam;
    PhotonPoseEstimator arducamePhotonPoseEstimator;
    PhotonPoseEstimator lifecamPhotonPoseEstimator;
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
        Arducam = new PhotonCamera("Arducam_OV9281_USB_Camera");
        Lifecam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

        //Cam mounted facing forward, half a meter forward of center, half a meter up from center. Change Both Later
        arducamRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 
        lifecamRobotToCam = new Transform3d(new Translation3d(0.152, -0.203, 0.4064), new Rotation3d(0,0, Math.PI)); 

        try 
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) 
        {
            e.printStackTrace();
        }

        arducamePhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Arducam, arducamRobotToCam);   
        lifecamPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, Lifecam, lifecamRobotToCam);   


        aprilTagFieldLayout.getTagPose(4).ifPresent(pose -> speakerPosition = pose.toPose2d()); //Get pose2d of speaker
    }

    public Optional<EstimatedRobotPose> getArducamPose(Pose2d prevEstimatedRobotPose) {
        arducamePhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return arducamePhotonPoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getLifecamPose(Pose2d prevEstimatedRobotPose) {
        lifecamPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return lifecamPhotonPoseEstimator.update();
    }

    public double getDistancetoSpeaker(Pose2d robotPose){
        distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, speakerPosition);
        return distanceToTarget;
    }

    public double getArducamYaw(){
        return arducamYaw;
    }

    public Pose2d getSpeakerPos() {
        return speakerPosition;
    }


    @Override
    public void periodic() {
        arducamResult = Arducam.getLatestResult();
        arducamHasTargets = arducamResult.hasTargets();

        if (arducamHasTargets)
        {
            arducamTargets = arducamResult.getTargets();
            arducamTarget = arducamResult.getBestTarget();
            arducamYaw = arducamTarget.getYaw();
            Logger.recordOutput("Arducam Yaw", arducamYaw);

            arducamPitch = arducamTarget.getPitch();
            Logger.recordOutput("Arducam Pitch", arducamPitch);
    
            arducamArea = arducamTarget.getArea();
            Logger.recordOutput("Arducam Area", arducamArea);

            arducamSkew = arducamTarget.getSkew();
            Logger.recordOutput("Arducam Skew", arducamSkew);

            arducamPose = arducamTarget.getBestCameraToTarget();
            arducamCorners = arducamTarget.getDetectedCorners();
        }

        lifecamResult = Lifecam.getLatestResult();
        lifecamHasTargets = lifecamResult.hasTargets();

        if (lifecamHasTargets)
        {
            lifecamTargets = lifecamResult.getTargets();
            lifecamTarget = lifecamResult.getBestTarget();

            lifecamYaw = lifecamTarget.getYaw();
            Logger.recordOutput("Lifecam Yaw", lifecamYaw);

            lifecamPitch = lifecamTarget.getPitch();
            Logger.recordOutput("Lifecam Pitch", lifecamPitch);
            
            lifecamArea = lifecamTarget.getArea();
            Logger.recordOutput("Lifecam Area", lifecamArea);

            lifecamSkew = lifecamTarget.getSkew();
            Logger.recordOutput("Lifecam Skew", lifecamSkew);

            lifecamPose = lifecamTarget.getBestCameraToTarget();
            lifecamCorners = lifecamTarget.getDetectedCorners();
        }
    }

}
