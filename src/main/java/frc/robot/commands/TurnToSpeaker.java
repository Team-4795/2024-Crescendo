package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class TurnToSpeaker {
    private static Vision vision = new Vision();
    private static PIDController rotationPID = new PIDController(0, 0, 0); // Change Values
    
    public static Command joystickDrive(Drive drive){
        return Commands.run(
        () -> {
            double deltaY = vision.getSpeakerPos().getY() - drive.getPose().getY();
            double angle = Math.asin(deltaY / vision.getDistancetoSpeaker(drive.getPose())) * 180/Math.PI;

            drive.drive(
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -rotationPID.calculate(vision.getYaw(), angle), 
                true, true);
        });
    }
}
