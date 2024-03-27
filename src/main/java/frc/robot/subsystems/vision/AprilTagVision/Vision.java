package frc.robot.subsystems.vision.AprilTagVision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.MAXSwerve.Drive;

public class Vision extends SubsystemBase {
    private VisionIO io[];
    private VisionIOInputsAutoLogged inputs[];

    private final Translation3d redSpeaker = new Translation3d(16.379342, 5.547868, 2.06);
    private final Translation3d blueSpeaker = new Translation3d(0.1619, 5.547868, 2.06);
    
    @AutoLogOutput
    private Translation3d speakerPosition = new Translation3d();

    public static Vision instance;
    
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

        inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        setSpeakerPos();
    }

    public double getDistancetoSpeaker(Pose2d robotPose) {
        if(speakerPosition == null){
            return 0;
        }
        return robotPose.getTranslation().getDistance(speakerPosition.toTranslation2d());
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
        for(int i = 0; i < io.length; i++) {
            io[i].setReferencePose(reference);
        }
    }

    public double getVisionStd(double distance) {
        return distance * 0.25;
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + VisionConstants.cameraIds[i], inputs[i]);
        }
        
        setSpeakerPos();

        for (int i = 0; i < io.length; i++) {
            for (int p = 0; p < inputs[i].pose.length; p++) {
                Pose3d robotPose = inputs[i].pose[p];

                List<Pose3d> tagPoses = new ArrayList<>();
                for (int tag : inputs[i].tags) {
                    VisionConstants.aprilTagFieldLayout.getTagPose(tag).ifPresent(tagPoses::add);
                }

                if (tagPoses.isEmpty()) continue;

                double distance = 0.0;
                for (var tag : tagPoses) {
                    distance += tag.getTranslation().getDistance(robotPose.getTranslation());
                }

                distance /= tagPoses.size();

                double xyStdDev = getVisionStd(distance) / tagPoses.size();
                var stddevs = VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(40));
                Drive.getInstance().addVisionMeasurement(robotPose.toPose2d(), inputs[i].timestamp[p], stddevs);
            }
        }
    }
}
