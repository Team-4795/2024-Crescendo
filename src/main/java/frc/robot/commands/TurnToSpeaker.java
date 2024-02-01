package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.OIConstants;

public class TurnToSpeaker {
    private static Vision vision = Vision.getInstance();
    private static PIDController rotationPID = new PIDController(0.1, 0, 0); // Change Values
    
    public static Command turnTowardsSpeaker(Drive drive){
        return Commands.run(
        () -> {
            double deltaY = vision.getSpeakerPos().getY() - drive.getPose().getY();
            // double angle = Math.asin(deltaY / vision.getDistancetoSpeaker(drive.getPose())) * 180/Math.PI;
            double angle = OIConstants.m_driverController.getRightY() * Math.PI;
            Logger.recordOutput("Angle", angle);

            drive.drive(
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                rotationPID.calculate(drive.getHeading(), angle), 
                true, true);
        }, drive);
    }
}
