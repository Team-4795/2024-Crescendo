package frc.robot.subsystems.MAXSwerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class GyroIONavx implements GyroIO {
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public void updateInputs(GyroIOInputs inputs) {
       //inputs.rollPositionRad = gyro.getRoll();
        //inputs.pitchPositionRad = gyro.getPitch();
        inputs.yawVelocity = gyro.getAngle();
    }

    public void reset() {
        gyro.reset();
    }

    public void setOffset(double offset) {
        gyro.setAngleAdjustment(offset);
    }
}