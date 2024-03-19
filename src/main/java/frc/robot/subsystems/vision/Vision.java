package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.EstimatedPose;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputsAutoLogged;

public class Vision extends SubsystemBase {
    private VisionIO io[];
    private VisionIOInputsAutoLogged inputs[];

    private Translation3d redSpeaker = new Translation3d(16.579342, 5.547868, 2);
    private Translation3d blueSpeaker = new Translation3d(-0.0381, 5.547868, 2);
    
    @AutoLogOutput
    private Translation3d speakerPosition = new Translation3d();

    public static Vision instance;
    
    public double distanceToTarget;

    public static Vision getInstance() {
        return instance;
    }

    public static Vision initialize(VisionIO... io) {
        if (instance == null) {
            instance = new Vision(io);
        }
        return instance;
    }

    private Vision(VisionIO visionIO[]) {
        io = visionIO;
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        setSpeakerPos();
    }

    public Optional<EstimatedPose> getCamPose(int camIndex) {
        return inputs[camIndex].visionPose;
    }

    public Optional<EstimatedPose> getGoldenBarrelPose() {
        return inputs.goldenBarrelPose;
    }

    public double getDistancetoSpeaker(Pose2d robotPose) {
        if(speakerPosition == null){
            return 0;
        }
        distanceToTarget = robotPose.getTranslation().getDistance(speakerPosition.toTranslation2d());
        return distanceToTarget;
    }

    public void setSpeakerPos(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                speakerPosition = redSpeaker;
            } else if (ally.get() == Alliance.Blue) {
                speakerPosition = blueSpeaker;
            }
        } else {
            speakerPosition = new Translation3d();
        }
    }

    public Translation3d getSpeakerPos() {
        return speakerPosition;
    }

    public void setReferencePose(Pose2d reference) {
        for(int i = 0; i < io.length; i++)
        {
            io[i].setReferencePose(reference);
        }
    }

    public double distanceToTag(int camIndex, int tag) {
        return io[camIndex].getDistanceToTag(tag);
    }

    public int numberOfTags(int camindex) {
        return inputs[camindex].numberOfTags;
    } 

    public int aprilTagDetected(int camIndex) {
        return inputs[camIndex].aprilTagDetected;
    }

    // public boolean lifeCamHastargets() {
    //     return inputs.lifeCamHastargets;
    // }

    // public double getLifecamYaw () {
    //     return inputs.lifeCamyaw;
    // }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Inst" + i, inputs[i]);
        }
        
        setSpeakerPos();
    }
}
