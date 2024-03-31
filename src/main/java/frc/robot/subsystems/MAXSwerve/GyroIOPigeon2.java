package frc.robot.subsystems.MAXSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOPigeon2 implements GyroIO{
    private Pigeon2 pigeon = new Pigeon2(DriveConstants.kPigeonCanId);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> pitch = pigeon.getPitch();

    private final StatusSignal<Double> rate = pigeon.getAngularVelocityZDevice();

    private final StatusSignal<Double> accelX = pigeon.getAccelerationX();
    private final StatusSignal<Double> accelY = pigeon.getAccelerationY();
    private final StatusSignal<Double> accelZ = pigeon.getAccelerationZ();

    private final double G = 9.80665;

    public GyroIOPigeon2(){
        pigeon.reset();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            yaw, 
            pitch,
            rate,
            accelX,
            accelY,
            accelZ);

        pigeon.optimizeBusUtilization(1.0);
    }

    @Override
    public void reset() {
        pigeon.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, pitch, rate, accelX, accelY, accelZ).isOK();
        inputs.yaw = Rotation2d.fromDegrees(pigeon.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
        inputs.yawVelocity = -Units.degreesToRadians(rate.getValueAsDouble()) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        inputs.pitch = pitch.getValueAsDouble();
        inputs.accel = new double[] {accelX.getValueAsDouble() * G, accelY.getValueAsDouble() * G, accelZ.getValueAsDouble() * G};
    }

}
