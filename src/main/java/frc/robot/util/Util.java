package frc.robot.util;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Util {
    public static final double kEpsilon = 1e-12;

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
        return epsilonEquals(twist.dx, other.dx)
            && epsilonEquals(twist.dy, other.dy)
            && epsilonEquals(twist.dtheta, other.dtheta);
    }

    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
        );
    }
}
