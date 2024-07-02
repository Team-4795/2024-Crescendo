package frc.robot.subsystems.vision.AprilTagVision;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.MAXSwerve.Drive;

public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim;
    TargetModel  targetModel;
    SimCameraProperties cameraProperties;
    PhotonCamera camera;
    PhotonCameraSim cameraSim;

    public VisionIOSim() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout);

        cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(78));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProperties.setCalibError(0.38, 0.2);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProperties.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        camera = new PhotonCamera("Barbary Fig");

        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        visionSim.addCamera(cameraSim, VisionConstants.cameraPoses[0]);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        cameraSim.enableDrawWireframe(true);
    }
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // if (Drive.getInstance() != null) {
            // Pose2d drivePose = Drive.getInstance().getPose();
            // inputs.barbaryFigPose = Optional.of(new EstimatedPose(drivePose, Timer.getFPGATimestamp()));
        // }
        visionSim.update(Drive.getInstance().getPose());

        visionSim.getDebugField();
    }
}
