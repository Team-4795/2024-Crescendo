package frc.robot.subsystems.MAXSwerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private double velocity = 0.0;
    private Rotation2d heading = new Rotation2d();

    @Override
    public void addOffset(Rotation2d change) {
        velocity = change.getRadians() / 0.02;
        heading = heading.plus(change);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = heading;
        inputs.yawVelocity = velocity;
    }

    @Override
    public void reset() {
        velocity = 0;
        heading = new Rotation2d();
    }
}