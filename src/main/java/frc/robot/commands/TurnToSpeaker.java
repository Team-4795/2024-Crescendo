package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.OIConstants;

public class TurnToSpeaker {
    private static Vision vision = Vision.getInstance();
    private static PIDController rotationPID = new PIDController(0.02, 0, 0.00025); // Change Values
    
    public static Command turnTowardsSpeaker(Drive drive){
        return Commands.run(
        () -> {
            Pose2d currentPose = drive.getPose();
            Translation2d velocity = drive.getTranslationVelocity();
            Pose2d newPose = new Pose2d(currentPose.getX() + (velocity.getX() * 0.02),
                                        currentPose.getY() + (velocity.getY() * 0.02), 
                                        currentPose.getRotation());
            double deltaY = vision.getSpeakerPos().getY() - newPose.getY();
            double angle = Math.asin(deltaY / vision.getDistancetoSpeaker(newPose)) * 180/Math.PI;
            double driveHeading = drive.getHeading();
            double output = rotationPID.calculate(driveHeading, angle);
            
            Logger.recordOutput("Vision/drive heading", driveHeading);
            Logger.recordOutput("Vision/angle", angle);
            Logger.recordOutput("Vision/output", output);

            drive.drive(
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.clamp(output, -1, 1), 
                true, true);
        }, drive);
    }
}
