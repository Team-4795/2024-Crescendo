package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;

public class AlignHeading {
    public static Drive drive = Drive.getInstance();
    public static final PIDController controller = new PIDController(0.025, 0, 0);

    public static Command align(double angle){
        
        controller.enableContinuousInput(0, 360);
        
        return Commands.run(() -> {
            double heading = drive.getHeading();
            heading = heading % 360;
            heading += (heading < 0) ? 360 : 0;

            Drive.getInstance().drive(
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband),
                -controller.calculate(heading, angle),
                true, true);

            Logger.recordOutput("Align/Heading", heading);
            Logger.recordOutput("Align/Angle", angle);
        }, drive);
    }
}
