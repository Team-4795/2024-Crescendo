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
    private VisionIO io;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private Translation3d redSpeaker = new Translation3d(16.579342, 5.547868, 2);
    private Translation3d blueSpeaker = new Translation3d(-0.0381, 5.547868, 2);
    
    @AutoLogOutput
    private Translation3d speakerPosition = new Translation3d();

    public static Vision instance;
    
    public double distanceToTarget;

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

        setSpeakerPos();
    }

    public Optional<EstimatedPose> getBarbaryFigPose() {
        return inputs.barbaryFigPose;
    }

    public Optional<EstimatedPose> getSaguaroPose() {
        return inputs.saguaroPose;
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
        io.setReferencePose(reference);
    }

    public double distanceToTag(int tag) {
        return io.getDistanceToTag(tag);
    }

    public int barbaryFigNumberOfTags() {
        return inputs.barbaryFigNumberOfTags;
    }

    public int barbaryFigAprilTagDetected() {
        return inputs.barbaryFigAprilTagDetected;
    }

    public int saguaroNumberOfTags() {
        return inputs.saguaroNumberOfTags;
    }

    public int saguaroAprilTagDetected() {
        return inputs.saguaroAprilTagDetected;
    }

    public int goldenBarrelNumberOfTags() {
        return inputs.goldenBarrelNumberOfTags;
    }

    public int goldenBarrelAprilTagDetected() {
        return inputs.goldenBarrelAprilTagDetected;
    }

    public boolean lifeCamHastargets() {
        return inputs.lifeCamHastargets;
    }

    public double getLifecamYaw () {
        return inputs.lifeCamyaw;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
        
        setSpeakerPos();
    }
}
