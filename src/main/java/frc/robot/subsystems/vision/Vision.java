package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
    //change camera name later
    PhotonCamera Camera = new PhotonCamera("photonvision");

    PhotonPipelineResult result = Camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();

    PhotonTrackedTarget target = result.getBestTarget();

    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    Transform3d pose = target.getBestCameraToTarget();
    List<TargetCorner> corners = target.getDetectedCorners();





}
