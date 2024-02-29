package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;

public class AlignHeading {
    public static Drive drive = Drive.getInstance();
    public static final ProfiledPIDController controller = new ProfiledPIDController(3, 0, 0.5, new Constraints(4, 6));

    // private static double goal = 0;
    public static Command align(double angle){
        controller.enableContinuousInput(0, 2 * Math.PI);

        return Commands.runOnce(() -> controller.reset(drive.getRotationHeading().getRadians(), drive.getTurnRate())).andThen(Commands.run(() -> {
            double heading = drive.getRotationHeading().getRadians();

            // double input = new Rotation2d(angle).minus(drive.getRotationHeading()).getRadians() * 0.5;
            double input = controller.calculate(heading, angle);
            Logger.recordOutput("Align/Velocity", input);

            Drive.getInstance().drive(
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband),
                input,
                true, true);

            Logger.recordOutput("Align/HeadingSetpoint", controller.getSetpoint().position);
            Logger.recordOutput("Align/VelocitySetpoint", controller.getSetpoint().velocity);
            Logger.recordOutput("Align/Heading", heading);
            Logger.recordOutput("Align/Angle", angle);

            
        }, drive));
    }

    // public static void setGoal(double angle) {
    //     goal = MathUtil.clamp(angle, 0, 2 * Math.PI);
    //     controller.setGoal(goal);
    // }
}
