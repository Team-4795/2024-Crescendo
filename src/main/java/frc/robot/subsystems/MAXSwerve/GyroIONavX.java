package frc.robot.subsystems.MAXSwerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public GyroIONavX() {
        reset();
    }
    
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yaw = gyro.getRotation2d();
        inputs.yawVelocity = gyro.getRate();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    public void setOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }
}