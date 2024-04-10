package frc.robot.subsystems.vision.intakeCam;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonTargetSortMode;
 
public interface IntakeCamVisionIO {
    @AutoLog
    public static class IntakeCamVisionIOInputs {
        public double noteYaw = 0;
        public double notePitch = 0;
        public double area = 0;
        public boolean hasTargets = false;
    }

    public default void setTargetComparator(PhotonTargetSortMode sortMode) {}

    public default void updateInputs(IntakeCamVisionIOInputs inputs) {}
} 

