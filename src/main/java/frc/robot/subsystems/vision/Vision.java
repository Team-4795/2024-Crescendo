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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    PhotonPipelineResult saguaroResult;
    PhotonPipelineResult barbaryFigResult;
    double saguaroYaw;
    double barbaryFigYaw;
    double saguaroPitch;
    double barbaryFigPitch;
    double saguaroArea;
    double barbaryFigArea;
    double saguaroSkew;
    double barbaryFigSkew;
    double distanceToTarget;
    Transform3d saguaroPose;
    Transform3d barbaryFigPose;
    List<TargetCorner> saguaroCorners;
    List<TargetCorner> barbaryFigCorners;
    boolean saguaroHasTargets;
    boolean barbaryFigHasTargets;
    List<PhotonTrackedTarget> saguaroTargets;
    List<PhotonTrackedTarget> barbaryFigTargets;
    PhotonTrackedTarget saguaroTarget;
    PhotonTrackedTarget barbaryFigTarget;

    PhotonCamera SaguaroCam;
    PhotonCamera BarbaryFig;

    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d saguaroRobotToCam;
    Transform3d barbaryFigRobotToCam;
    PhotonPoseEstimator saguaroPhotonPoseEstimator;
    PhotonPoseEstimator barbaryFigPhotonPoseEstimator;
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
        SaguaroCam = new PhotonCamera("Saguaro");
        BarbaryFig = new PhotonCamera("Barbary Fig");

        //Cam mounted facing forward, half a meter forward of center, half a meter up from center. Change Both Later
        saguaroRobotToCam = new Transform3d(new Translation3d(0.241, 0.0889, 0.826), new Rotation3d(0,0,0)); 
        barbaryFigRobotToCam = new Transform3d(new Translation3d(-0.178, 0.165, 0.489), new Rotation3d(0,0, Math.PI)); 

        try 
        {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } 
        catch (IOException e) 
        {
            e.printStackTrace();
        }

        saguaroPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, SaguaroCam, saguaroRobotToCam);   
        barbaryFigPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, BarbaryFig, barbaryFigRobotToCam);   


        Optional<Alliance> ally = DriverStation.getAlliance();
        aprilTagFieldLayout.getTagPose(7).ifPresent(pose -> speakerPosition = pose.toPose2d());
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                aprilTagFieldLayout.getTagPose(4).ifPresent(pose -> speakerPosition = pose.toPose2d()); //Get pose2d of speaker
            }
            else if (ally.get() == Alliance.Blue) {
                aprilTagFieldLayout.getTagPose(7).ifPresent(pose -> speakerPosition = pose.toPose2d()); //Get pose2d of speaker

            }
        }
        }

    public Optional<EstimatedRobotPose> getArducamPose(Pose2d prevEstimatedRobotPose) {
        saguaroPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return saguaroPhotonPoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getLifecamPose(Pose2d prevEstimatedRobotPose) {
        barbaryFigPhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return barbaryFigPhotonPoseEstimator.update();
    }

    public double getDistancetoSpeaker(Pose2d robotPose){
        distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, speakerPosition);
        return distanceToTarget;
    }

    public double getSaguaroYaw(){
        return saguaroYaw;
    }

    public Pose2d getSpeakerPos() {
        return speakerPosition;
    }


    @Override
    public void periodic() {
        saguaroResult = SaguaroCam.getLatestResult();
        saguaroHasTargets = saguaroResult.hasTargets();

        if (saguaroHasTargets)
        {
            saguaroTargets = saguaroResult.getTargets();
            saguaroTarget = saguaroResult.getBestTarget();

            saguaroYaw = saguaroTarget.getYaw();
            Logger.recordOutput("Arducam Yaw", saguaroYaw);

            saguaroPitch = saguaroTarget.getPitch();
            Logger.recordOutput("Arducam Pitch", saguaroPitch);
    
            saguaroArea = saguaroTarget.getArea();
            Logger.recordOutput("Arducam Area", saguaroArea);

            saguaroSkew = saguaroTarget.getSkew();
            Logger.recordOutput("Arducam Skew", saguaroSkew);

            

            saguaroPose = saguaroTarget.getBestCameraToTarget();
            saguaroCorners = saguaroTarget.getDetectedCorners();
        }

        barbaryFigResult = BarbaryFig.getLatestResult();
        barbaryFigHasTargets = barbaryFigResult.hasTargets();

        if (barbaryFigHasTargets)
        {
            barbaryFigTargets = barbaryFigResult.getTargets();
            barbaryFigTarget = barbaryFigResult.getBestTarget();

            barbaryFigYaw = barbaryFigTarget.getYaw();
            Logger.recordOutput("Lifecam Yaw", barbaryFigYaw);

            barbaryFigPitch = barbaryFigTarget.getPitch();
            Logger.recordOutput("Lifecam Pitch", barbaryFigPitch);
            
            barbaryFigArea = barbaryFigTarget.getArea();
            Logger.recordOutput("Lifecam Area", barbaryFigArea);

            barbaryFigSkew = barbaryFigTarget.getSkew();
            Logger.recordOutput("Lifecam Skew", barbaryFigSkew);

            Logger.recordOutput("speaker pos", speakerPosition);


            barbaryFigPose = barbaryFigTarget.getBestCameraToTarget();
            barbaryFigCorners = barbaryFigTarget.getDetectedCorners();
        }
    }

}
