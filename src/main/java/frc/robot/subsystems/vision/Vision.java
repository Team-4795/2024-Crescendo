package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonVersion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public static Vision instance;
    
    public double distanceToTarget;
    public Pose2d speakerPosition;

    public PhotonPoseEstimator saguaroPhotonPoseEstimator;
    public PhotonPoseEstimator barbaryFigPhotonPoseEstimator;

    public static Vision getInstance() {
        return instance;
    }

    public static Vision initialize(VisionIO io) {
        if (instance == null) {
            instance = new Vision(io);
        }
        return instance;
    }


    private Vision(VisionIO visionIO) {
        io = visionIO;
        io.updateInputs(inputs);

        speakerPosition = io.getSpeakerPos();
    }

    public Optional<EstimatedRobotPose> getBarbaryFigPose(Pose2d prevEstimatedRobotPose) {
        return io.getBarbaryFigPose(prevEstimatedRobotPose);
    }

    public Optional<EstimatedRobotPose> getSaguaroPose(Pose2d prevEstimatedRobotPose) {
        return io.getSaguaroPose(prevEstimatedRobotPose);
    }

    public double getDistancetoSpeaker(Pose2d robotPose) {
        if(speakerPosition == null){
            return 0;
        }
        distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, speakerPosition);
        return distanceToTarget;
    }

    public Pose2d getSpeakerPos(){
        speakerPosition = io.getSpeakerPos();
        return speakerPosition;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

}
