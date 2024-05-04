package frc.robot.subsystems.vision.intakeCam;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonTargetSortMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.StagingLocations;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.AprilTagVision.VisionConstants;

public class IntakeCamVision extends SubsystemBase {
    private static IntakeCamVisionIO io;
    private IntakeCamVisionIOInputsAutoLogged inputs = new IntakeCamVisionIOInputsAutoLogged();

    private static IntakeCamVision instance;

    public static IntakeCamVision getInstance() {
        return instance;
    }

    public static IntakeCamVision initialize(IntakeCamVisionIO io) {
        if (instance == null) {
            instance = new IntakeCamVision(io);
        }
        return instance;
    }

    private IntakeCamVision(IntakeCamVisionIO intakeCamVisionIO) {
        io = intakeCamVisionIO;
    }

    public double getIntakeCamYaw() {
        return inputs.noteYaw;
    }

    public boolean intakeCamHasTargets() {
        return inputs.hasTargets;
    }

    public void setTargetComparator(PhotonTargetSortMode sortMode){
        io.setTargetComparator(sortMode);
    }

    public boolean isNoteInFront(int note) {
        Pose2d robotPose = Drive.getInstance().getPose();
        Translation2d cameraPose = new Translation2d(
            robotPose.getX() + robotPose.getRotation().getCos() * VisionConstants.intakeCamOffset, 
            robotPose.getY() + robotPose.getRotation().getSin() * VisionConstants.intakeCamOffset);
        Translation2d notePose = StagingLocations.centerlineTranslations[note];
        double distance = cameraPose.getDistance(notePose);
        double yChange = notePose.getY() - cameraPose.getY();
        double yaw = 0.0;
        double error = 0.0;
        if(Constants.alliance == Alliance.Red){
            yaw = Math.asin(yChange / distance) + Drive.getInstance().getWrappedHeading();
            error = Math.abs(Units.radiansToDegrees(yaw) - this.getIntakeCamYaw());
        } else {
            yaw = Math.asin(yChange / distance) - Drive.getInstance().getWrappedHeading();
            error = Math.abs(Units.radiansToDegrees(yaw) + this.getIntakeCamYaw()); //because photon and robot yaw coordinate systems are flipped on blue
        }
        Logger.recordOutput("Intake Cam/Camera pose", cameraPose);
        Logger.recordOutput("Intake Cam/Note " + note + "/ yaw", Units.radiansToDegrees(yaw));
        Logger.recordOutput("Intake Cam/Note " + note + "/ Unaltered expected yaw", Math.asin(yChange / distance));
        Logger.recordOutput("Intake Cam/Note " + note + "/ Distance", distance);
        Logger.recordOutput("Intake Cam/Note " + note + "/ delta Y", yChange);
        Logger.recordOutput("Intake Cam/Note " + note + "/ error degrees", error);

        return this.intakeCamHasTargets() && error < VisionConstants.degreeTolerance; 
    }

    public double getDistanceToNote(int note){
        Pose2d robotPose = Drive.getInstance().getPose();
        Translation2d cameraPose = new Translation2d(
            robotPose.getX() + robotPose.getRotation().getCos() * VisionConstants.intakeCamOffset, 
            robotPose.getY() + robotPose.getRotation().getSin() * VisionConstants.intakeCamOffset);
        Translation2d notePose = StagingLocations.centerlineTranslations[note];
        return cameraPose.getDistance(notePose);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Cam", inputs);
        Logger.recordOutput("Intake Cam/Note in Front 4", this.isNoteInFront(4));
        Logger.recordOutput("Intake Cam/ Note in Front 5", this.isNoteInFront(5));
        Logger.recordOutput("Auto Notes/ Note Position 4", StagingLocations.centerlineTranslations[4]);
    }
    
}
