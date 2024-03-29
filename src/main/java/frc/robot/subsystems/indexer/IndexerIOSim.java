package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO{
    DCMotorSim motor = new DCMotorSim(DCMotor.getNeo550(1), 5, 0.001);

    @Override
    public void setIndexerSpeed(double speed) {
        motor.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        motor.update(0.02);
        inputs.bottomMotorSpeed = motor.getAngularVelocityRPM();
    }
}
