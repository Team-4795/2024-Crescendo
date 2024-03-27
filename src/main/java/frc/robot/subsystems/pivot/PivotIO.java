package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
 
public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double pivotInputVolts = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotPositionRads = 0.0;
        public double pivotVelocityRads = 0.0;
        public double pivotCurrent = 0.0;

        public double pivotMotorVelocityRadPerSec = 0.0;
        public double pivotMotorPositionRads = 0.0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void rotatePivot(double speed) {}

    public default void setVoltage(double volts) {}   
    
    // True is brake, false is coast
    public default void setIdleMode(boolean mode) {}
} 

