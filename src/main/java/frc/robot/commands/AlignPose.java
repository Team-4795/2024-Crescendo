package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;

public class AlignPose{
    
    private static double mult = 1;
    private static Alliance alliance;

    private static double driveHeading;
    private static double deltaAngle;
    private static double previousAngle;
    private static double deltaY;
    private static double deltaX;
    private static double desiredAngle;

    private static double omega;
    private static double output;

    private static Pose2d targetPose;
    private static Pose2d currentPose;
    private static Pose2d augmentedPose;
    private static boolean inverted; //intake facing or shooter facing (true for shooter)
    private static Translation2d velocity;
    private static final PIDController rotationPID = new PIDController(0.09, 0, 0);

    public static void setTarget(Pose2d target, boolean invert, Alliance ally) {
        targetPose = target;
        inverted = invert;
        alliance = ally;
        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(5);
        initialize();
    }

    private static void initialize(){
        DriverStation.getAlliance().ifPresent(alliance -> mult = (alliance == Alliance.Red) ? -1.0 : 1.0);
        currentPose = Drive.getInstance().getPose();
        velocity = Drive.getInstance().getTranslationVelocity();
        augmentedPose = new Pose2d(currentPose.getX() + (velocity.getX() * 0.02),
                currentPose.getY() + (velocity.getY() * 0.02),
                currentPose.getRotation());
        deltaY = targetPose.getY() - augmentedPose.getY();
        deltaX = targetPose.getX() - augmentedPose.getX();
        deltaX *= (inverted) ? -mult : mult;
        deltaY *= (inverted) ? -mult : mult;
        previousAngle = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));
    }

    public static double calculateRotationSpeed() {        
        currentPose = Drive.getInstance().getPose();
        velocity = Drive.getInstance().getTranslationVelocity();
        augmentedPose = new Pose2d(currentPose.getX() + (velocity.getX() * 0.02),
                currentPose.getY() + (velocity.getY() * 0.02),
                currentPose.getRotation());

        deltaY = targetPose.getY() - augmentedPose.getY();
        deltaX = targetPose.getX() - augmentedPose.getX();
        deltaX *= (inverted) ? -mult : mult;
        deltaY *= (inverted) ? -mult : mult;
        // deltaX *= (alliance == Alliance.Red) ? -1 : 1;
        // deltaY *= (alliance == Alliance.Red) ? -1 : 1;
        desiredAngle = Units.radiansToDegrees(Math.atan2(deltaY, deltaX));

        deltaAngle = Units.degreesToRadians(desiredAngle - previousAngle);
        omega = deltaAngle / 0.02;

        driveHeading = Drive.getInstance().getWrappedHeading();
        output = omega + rotationPID.calculate(driveHeading, desiredAngle);

        previousAngle = desiredAngle;

        Drive.getInstance().setAtTarget(Optional.of(rotationPID.atSetpoint()));
        
        return MathUtil.clamp(output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed);
    }

    public static double getDistanceToTarget(){
        return targetPose.getTranslation().getDistance(currentPose.getTranslation());
    }

    public static void periodic(){
        Logger.recordOutput("Pose Align/drive heading", driveHeading);
        Logger.recordOutput("Pose Align/desired angle", desiredAngle);
        Logger.recordOutput("Pose Align/output", output);
        Logger.recordOutput("Pose Align/augmentedPose", augmentedPose);
        Logger.recordOutput("Pose Align/targetPose", targetPose);
        Logger.recordOutput("Pose Align/previous angle", previousAngle);
        Logger.recordOutput("Pose Align/omega", omega);
    }
}
