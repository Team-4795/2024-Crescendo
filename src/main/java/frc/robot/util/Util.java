package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

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

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    }

    public class AllianceFlipUtil {
        /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
        public static double apply(double xCoordinate) {
            if (shouldFlip()) {
                return FieldConstants.fieldLength - xCoordinate;
            } else {
                return xCoordinate;
            }
        }

        /** Flips a translation to the correct side of the field based on the current alliance color. */
        public static Translation2d apply(Translation2d translation) {
            if (shouldFlip()) {
                return new Translation2d(apply(translation.getX()), translation.getY());
            } else {
                return translation;
            }
        }

        /** Flips a rotation based on the current alliance color. */
        public static Rotation2d apply(Rotation2d rotation) {
            if (shouldFlip()) {
                return new Rotation2d(-rotation.getCos(), rotation.getSin());
            } else {
                return rotation;
            }
        }

        /** Flips a pose to the correct side of the field based on the current alliance color. */
        public static Pose2d apply(Pose2d pose) {
            if (shouldFlip()) {
                return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
            } else {
                return pose;
            }
        }

        public static Translation3d apply(Translation3d translation3d) {
            if (shouldFlip()) {
                return new Translation3d(
                    apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
            } else {
                return translation3d;
            }
        }
    }
}