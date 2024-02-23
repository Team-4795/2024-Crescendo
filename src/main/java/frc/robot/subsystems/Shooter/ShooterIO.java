package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double topShooterMotorVelocityRPM = 0.0;
        public double topShooterMotorAppliedVolts = 0.0;
        public double topShooterCurrent = 0.0;
        public double topShooterAppliedVolts = 0.0;

        public double bottomShooterMotorVelocityRPM = 0.0;
        public double bottomShooterMotorAppliedVolts = 0.0;
        public double bottomShooterCurrent = 0.0;
        public double bottomShooterAppliedVolts = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void runShooterMotorsRPM(double topSpeed, double bottomSpeed) {}

    public default void runVoltageTop(double volts) {}

    public default void runVoltageBottom(double volts) {}
}
