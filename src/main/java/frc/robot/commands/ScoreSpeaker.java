package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateManager;
import frc.robot.Constants.OIConstants;
import frc.robot.StateManager.State;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ScoreSpeaker extends Command {

    public final double speakerHeight = 1.98;
    public final double pivotHeight = 0.2794;
    public double distanceToSpeaker = 0.0;
    public double angleCalc = 0.0;
    private Vision vision = Vision.getInstance();
    private Drive drive = Drive.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private PIDController rotationPID = new PIDController(1, 0, 0); // Change Values

    public ScoreSpeaker() {
        addRequirements(Pivot.getInstance(), drive, vision, Shooter.getInstance());
        StateManager.getInstance().setState(State.ScoreSpeaker);

    }

    @Override
    public void execute() {
        // called every 20 ms
        distanceToSpeaker = vision.getDistancetoSpeaker(drive.getPose());
        angleCalc = Math.atan((speakerHeight - pivotHeight) / distanceToSpeaker);
        Pivot.getInstance().setGoal(angleCalc);

        double deltaY = vision.getSpeakerPos().getY() - drive.getPose().getY();
        double angle = Math.asin(deltaY / distanceToSpeaker) * 180 / Math.PI;
        Logger.recordOutput("Angle", angle);

        drive.drive(
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband),
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband),
                -rotationPID.calculate(drive.getHeading(), angle),
                true, true);
    
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}