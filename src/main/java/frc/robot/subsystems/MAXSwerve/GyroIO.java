package frc.robot.subsystems.MAXSwerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yaw = new Rotation2d();
        public double yawVelocity = 0.0; //degrees per second
        public double[] accel = new double[] {0, 0, 0};
    }

    public default void updateInputs(GyroIOInputs inputs){}
    
    public default void reset( ){}

    public default void addOffset(Rotation2d change) {}
}
