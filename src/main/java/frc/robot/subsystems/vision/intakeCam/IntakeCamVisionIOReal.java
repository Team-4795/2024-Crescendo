package frc.robot.subsystems.vision.intakeCam;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.AprilTagVision.VisionConstants;

public class IntakeCamVisionIOReal implements IntakeCamVisionIO{
    private PhotonCamera camera;
    private PhotonPipelineResult intakeCamResult;
    private double lastDetectedTimestamp = 0;

    public IntakeCamVisionIOReal() {
        camera = new PhotonCamera("Queen of the Night"); 
    }

    @Override
    public void updateInputs(IntakeCamVisionIOInputs inputs){
        intakeCamResult = camera.getLatestResult();
        inputs.hasTargets = intakeCamResult.hasTargets();
        
        
        if (inputs.hasTargets) {
            PhotonTrackedTarget intakeCamTarget = intakeCamResult.getBestTarget();
            inputs.camYaw = intakeCamTarget.getYaw();
            inputs.area = intakeCamTarget.getArea();
            lastDetectedTimestamp = intakeCamResult.getTimestampSeconds();
        } else if (DriverStation.isTeleopEnabled()){
            if(inputs.area > VisionConstants.areaCutoff && withinDelayTolerance() && !Intake.getInstance().isIntaking() && !Indexer.getInstance().isStoring()){
                inputs.hasTargets = true;
            }
        }
    }

    private boolean withinDelayTolerance(){
        return (intakeCamResult.getTimestampSeconds() - lastDetectedTimestamp) < VisionConstants.timeDelay;
    }
}
