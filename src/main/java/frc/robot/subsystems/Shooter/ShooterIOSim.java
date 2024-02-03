package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO{
    DCMotorSim motor = new DCMotorSim(DCMotor.getNeoVortex(1), 4, 0.001);
    private double appliedVolts = 0.0;

    @Override
    public void runShooterMotors(double speed) {
        appliedVolts = MathUtil.clamp(12 * speed, -12, 12);
        motor.setInputVoltage(appliedVolts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        motor.update(0.02);
        inputs.shooterMotorAppliedVolts = appliedVolts;
        inputs.shooterMotorVelocityRPM = motor.getAngularVelocityRPM();
    }
}
