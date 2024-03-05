package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.MAXSwerve.Drive;

public class VisionIOSim implements VisionIO {
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // if (Drive.getInstance() != null) {
            // Pose2d drivePose = Drive.getInstance().getPose();
            // inputs.barbaryFigPose = Optional.of(new EstimatedPose(drivePose, Timer.getFPGATimestamp()));
        // }
    }
}
