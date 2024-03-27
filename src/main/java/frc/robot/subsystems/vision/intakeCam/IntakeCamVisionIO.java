package frc.robot.subsystems.vision.intakeCam;

import org.littletonrobotics.junction.AutoLog;
 
public interface IntakeCamVisionIO {
    @AutoLog
    public static class IntakeCamVisionIOInputs {
        public double camYaw = 0;
        public double area = 0;
        public boolean hasTargets = false;
    }

    public default void updateInputs(IntakeCamVisionIOInputs inputs) {}
} 

