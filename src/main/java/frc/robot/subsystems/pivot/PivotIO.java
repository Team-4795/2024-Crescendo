package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
 
public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotPositionRads = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void rotatePivot(double speed) {}

    public default void setVoltage(double volts) {}        
} 

