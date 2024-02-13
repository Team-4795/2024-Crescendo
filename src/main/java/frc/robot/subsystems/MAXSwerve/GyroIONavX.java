package frc.robot.subsystems.MAXSwerve;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavx implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public GyroIONavx() {
        reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
       //inputs.rollPositionRad = gyro.getRoll();
        //inputs.pitchPositionRad = gyro.getPitch();
        inputs.yaw = Rotation2d.fromDegrees(-gyro.getAngle());
        inputs.yawVelocity = gyro.getRate();
    }

    public void reset() {
        gyro.reset();
    }

    public void setOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }
}