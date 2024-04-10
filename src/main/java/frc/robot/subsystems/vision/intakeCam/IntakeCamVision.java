package frc.robot.subsystems.vision.intakeCam;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonTargetSortMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

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

    public boolean isNoteInFront(Translation2d robotOdometry, Translation2d notePose, double photonVisionYaw) {
        double distance = robotOdometry.getDistance(notePose);
        double yChange = robotOdometry.getY() - notePose.getY();

        double angle = robotOdometry.getAngle().getDegrees();

        double yaw = Math.asin(yChange / distance) - angle;

        return Math.abs(yaw - photonVisionYaw) < 5;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake Cam", inputs);

        // if (inputs.hasTargets) {
        //     double camZ = Units.inchesToMeters(9.35);
        //     double camY = Units.inchesToMeters(2.516);
        //     double angle = Units.degreesToRadians(20.837);
        // }
    }
    
}
