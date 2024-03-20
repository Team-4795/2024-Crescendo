package frc.robot.subsystems.vision.intakeCam;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IntakeCamVisionIOReal implements IntakeCamVisionIO{
    PhotonCamera camera;

    private IntakeCamVisionIOReal() {
        camera = new PhotonCamera("Queen of the Night"); 
    }

    @Override
    public void updateInputs(IntakeCamVisionIOInputs inputs){
        PhotonPipelineResult intakeCamResult = camera.getLatestResult();
        inputs.hasTargets = intakeCamResult.hasTargets();
        
        if (inputs.hasTargets) {
            PhotonTrackedTarget intakeCamTarget = intakeCamResult.getBestTarget();
            inputs.camYaw = intakeCamTarget.getYaw();
        }
    }
}
