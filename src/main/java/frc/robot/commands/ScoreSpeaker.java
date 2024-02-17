package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateManager;
import frc.robot.Constants.OIConstants;
import frc.robot.StateManager.State;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
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
    private Pivot pivot = Pivot.getInstance();
    private PIDController rotationPID = new PIDController(0.09, 0, 0); // Change Values

    public ScoreSpeaker() {
        addRequirements(pivot, drive, vision);
        rotationPID.enableContinuousInput(-180, 180);
        StateManager.getInstance().setState(State.ScoreSpeaker);
        StateManager.getInstance().setMutable(false);
    }

    @Override
    public void execute() {
        // called every 20 ms
        distanceToSpeaker = vision.getDistancetoSpeaker(drive.getPose());
        angleCalc = Math.atan((speakerHeight - pivotHeight) / distanceToSpeaker);
        Pivot.getInstance().setGoal(angleCalc);

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

        drive.runVelocity(new ChassisSpeeds(
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband)
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband)
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                MathUtil.clamp(omega - output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed)));

        
        Logger.recordOutput("Vision/drive heading", driveHeading);
        Logger.recordOutput("Speaker Position", vision.getSpeakerPos());
        Logger.recordOutput("Vision/angle", angle);
        Logger.recordOutput("Vision/output", output);
        Logger.recordOutput("NewPose", newPose);
        Logger.recordOutput("Vision/desired angle", desiredAngle);
        Logger.recordOutput("Vision/omega", omega);
    }

    @Override
    public void end(boolean interuppted){
        StateManager.getInstance().setMutable(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
