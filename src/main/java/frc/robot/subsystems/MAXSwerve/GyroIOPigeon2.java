package frc.robot.subsystems.MAXSwerve;

import com.ctre.phoenix6.hardware.Pigeon2;

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
        inputs.yaw = pigeon.getAngle();
        inputs.yawVelocity = pigeon.getRate();
    }
    
}
