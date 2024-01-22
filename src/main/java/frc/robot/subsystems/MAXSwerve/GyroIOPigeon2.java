package frc.robot.subsystems.MAXSwerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO{

    private Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonCanId);

    public GyroIOPigeon2(){
        pigeon.reset();
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = Rotation2d.fromDegrees(pigeon.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
        inputs.yawVelocity = pigeon.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    
}
