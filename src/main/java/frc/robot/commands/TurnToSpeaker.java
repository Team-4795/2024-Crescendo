package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.OIConstants;

public class TurnToSpeaker {
    private static Vision vision = Vision.getInstance();
    private static PIDController rotationPID = new PIDController(0.09, 0, 0.0); // Change Values
    public static void initialize(){
        // rotationPID.enableContinuousInput(-180, 180);
    }
    public static Command turnTowardsSpeaker(Drive drive){
        return Commands.run(
        () -> {
            Pose2d currentPose = drive.getPose();
            Translation2d velocity = drive.getTranslationVelocity();
            Pose2d newPose = new Pose2d(currentPose.getX() + (velocity.getX() * 0.02),
                                        currentPose.getY() + (velocity.getY() * 0.02), 
                                        currentPose.getRotation());
            
            double deltaY = vision.getSpeakerPos().getY() - newPose.getY();
            double realdeltaY = vision.getSpeakerPos().getY() - currentPose.getY();
            double desiredAngle = Units.radiansToDegrees(Math.asin(realdeltaY / vision.getDistancetoSpeaker(currentPose)));
            double angle = Units.radiansToDegrees(Math.asin(deltaY / vision.getDistancetoSpeaker(newPose)));

            double deltaAngle = Units.degreesToRadians(angle - desiredAngle);
            double omega = -deltaAngle / 0.02;

            double driveHeading = drive.getWrappedHeading();
            double output = rotationPID.calculate(driveHeading, angle);
            
            Logger.recordOutput("Vision/drive heading", driveHeading);
            Logger.recordOutput("Speaker Position", vision.getSpeakerPos());
            Logger.recordOutput("Vision/angle", angle);
            Logger.recordOutput("Vision/output", output);
            Logger.recordOutput("NewPose", newPose);
            Logger.recordOutput("Vision/desired angle", desiredAngle);
            Logger.recordOutput("Vision/omega", omega);

            drive.runVelocity(new ChassisSpeeds(
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
                MathUtil.clamp(omega - output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed)
            ));
        }, drive);
    }
}
