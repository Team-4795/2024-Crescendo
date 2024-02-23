package frc.robot.subsystems.MAXSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon2 implements GyroIO{
    private Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonCanId);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> rate = pigeon.getAngularVelocityZDevice();
    // private final StatusSignal<Double> rate = pigeon.getAngularVelocityZDevice();

    public GyroIOPigeon2(){
        pigeon.reset();

        BaseStatusSignal.setUpdateFrequencyForAll(50, yaw, rate);

        pigeon.optimizeBusUtilization(1.0);
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, rate).isOK();
        inputs.yaw = Rotation2d.fromDegrees(yaw.getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
        inputs.yawVelocity = rate.getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

}
