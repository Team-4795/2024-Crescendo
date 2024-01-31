package frc.robot.subsystems.MAXSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private Rotation2d heading = new Rotation2d();

    @Override
    public void addOffset(Rotation2d change) {
        heading = heading.plus(change);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = heading;
    }

    @Override
    public void reset() {
        heading = new Rotation2d();
    }
}