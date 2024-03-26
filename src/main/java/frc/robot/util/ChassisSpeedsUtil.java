package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtil {
    public static ChassisSpeeds correctForDynamics(
        ChassisSpeeds input, double loopPeriodSecs, double driftRate
    ) {
        Pose2d desiredPose = new Pose2d(
            input.vxMetersPerSecond * loopPeriodSecs,
            input.vyMetersPerSecond * loopPeriodSecs,
            Rotation2d.fromRadians(input.omegaRadiansPerSecond * loopPeriodSecs * driftRate)
        );

        var twist = new Pose2d().log(desiredPose);

        return new ChassisSpeeds(
            twist.dx / loopPeriodSecs,
            twist.dy / loopPeriodSecs,
            input.omegaRadiansPerSecond
        );
    }
}
