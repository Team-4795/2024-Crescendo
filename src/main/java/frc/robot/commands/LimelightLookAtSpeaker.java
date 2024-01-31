package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.util.LimelightHelpers;

public class LimelightLookAtSpeaker {

    private static PIDController rotationPID = new PIDController(1, 0, 0); // Change values

    public static Command lookAtSpeaker(Drive drive) {
        return Commands.run(
                () -> {
                    double x = LimelightHelpers.getTX("limelight");
                    drive.drive(
                            -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftY(),
                                    OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(OIConstants.m_driverController.getLeftX(),
                                    OIConstants.kDriveDeadband),
                            -rotationPID.calculate(x, 0),
                            true, true);
                },
                drive);
    }

}
