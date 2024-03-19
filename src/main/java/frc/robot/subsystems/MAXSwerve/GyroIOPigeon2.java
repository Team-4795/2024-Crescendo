package frc.robot.subsystems.MAXSwerve;

import java.util.OptionalDouble;
import java.util.Queue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO{
    private Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonCanId);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> rate = pigeon.getAngularVelocityZDevice();
    private Queue<Double> timestamps;
    private Queue<Double> rotationQueue;

    public GyroIOPigeon2(){
        pigeon.reset();

        timestamps = OdometryThread.getInstance().registerTimestamps();
        rotationQueue = OdometryThread.getInstance().registerSignal(() -> {
            if(yaw.refresh().getStatus().isOK()){
                return OptionalDouble.of(pigeon.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
            } else {
                return OptionalDouble.empty();
            }
        });

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
        inputs.yaw = Rotation2d.fromDegrees(pigeon.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
        inputs.yawVelocity = -Units.degreesToRadians(rate.getValueAsDouble()) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

        inputs.timestamps = timestamps.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYaw = rotationQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
        timestamps.clear();
        rotationQueue.clear();
    }
}
