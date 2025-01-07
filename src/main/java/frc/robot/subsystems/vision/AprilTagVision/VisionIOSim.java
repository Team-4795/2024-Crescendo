package frc.robot.subsystems.vision.AprilTagVision;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
        cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(78));
        cameraProperties.setCalibError(0.38, 0.2);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        camera = new PhotonCamera("Jermaine Coral");
        // camera2 = new PhotonCamera("Kendrick LaBarge");

        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        // Front sideways Camera
        /*
        visionSim.addCamera(
            cameraSim, 
            new Transform3d(
                new Translation3d(
                    0.3,
                    0,
                    Units.inchesToMeters(7)), 
                new Rotation3d(
                    Units.degreesToRadians(90), 
                    Units.degreesToRadians(-30), 
                    0)));
                */
        

        // Backwards sideways Camera
        /* 
        visionSim.addCamera(
            cameraSim, 
            new Transform3d(
                new Translation3d(
                    -0.3,
                    0,
                    Units.inchesToMeters(7)), 
                new Rotation3d(
                    Units.degreesToRadians(90), 
                    Units.degreesToRadians(-30), 
                    Units.degreesToRadians(180))));
                */

        //Back left Normal Camera
        visionSim.addCamera(
            cameraSim, 
            new Transform3d(
                new Translation3d(
                    -0.375,
                    -0.375,
                    Units.inchesToMeters(7)), 
                new Rotation3d(
                    0, 
                    Units.degreesToRadians(-45), 
                    135)));

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
