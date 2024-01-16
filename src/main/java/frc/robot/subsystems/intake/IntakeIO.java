package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocity;
        public double voltage;

    }
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setMotorSpeed(double speed){}
}
