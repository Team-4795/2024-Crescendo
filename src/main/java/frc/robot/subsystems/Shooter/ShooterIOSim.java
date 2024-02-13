package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    DCMotorSim motor = new DCMotorSim(DCMotor.getKrakenX60(2), 1, 0.001);
    private double appliedVolts = 0.0;

    @Override
    public void runShooterMotorsRPM(double topSpeed, double bottomShooterMotor) {
        // appliedVolts = MathUtil.clamp(12 * topSpeed, -12, 12);
        // motor.setInputVoltage(appliedVolts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        motor.update(0.02);
        inputs.topShooterMotorAppliedVolts = appliedVolts;
        inputs.topShooterMotorVelocityRPM = motor.getAngularVelocityRPM();
        inputs.bottomShooterMotorAppliedVolts = appliedVolts;
        inputs.bottomShooterMotorVelocityRPM = motor.getAngularVelocityRPM();
    }
}
