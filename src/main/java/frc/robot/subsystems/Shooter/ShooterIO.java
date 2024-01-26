package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterMotorVelocityRadPerSec = 0.0;
        public double shooterMotorAppliedVolts = 0.0;
        
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void runShooterMotors(double speed) {}

    
}
