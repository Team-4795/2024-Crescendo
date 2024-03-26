package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
import frc.robot.subsystems.vision.AprilTagVision.Vision;

import java.util.Optional;

public class AlignSpeaker extends Command {

    private double previousAngle;
    private double mult;

    private Vision vision = Vision.getInstance();
    private Drive drive = Drive.getInstance();
    private PIDController rotationPID = new PIDController(34*0.6, 0, 34*0.49/8);

    public AlignSpeaker() {
        addRequirements(drive);

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize(){
        DriverStation.getAlliance().ifPresent(alliance -> mult = (alliance == Alliance.Red) ? -1.0 : 1.0);
        // rotationPID.reset(drive.getRotationHeading().getRadians(), drive.getTurnRate());

        Pose2d currentPose = drive.getPose();
        Translation2d velocity = drive.getTranslationVelocity();
        Pose2d newPose = new Pose2d(currentPose.getX() + (velocity.getX() * 0.02),
                currentPose.getY() + (velocity.getY() * 0.02),
                currentPose.getRotation());
        double deltaY = vision.getSpeakerPos().getY() - newPose.getY();
        previousAngle = mult * Math.asin(deltaY / vision.getDistancetoSpeaker(newPose));
        
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();
        Translation2d velocity = drive.getTranslationVelocity();
        Pose2d newPose = new Pose2d(currentPose.getX() + (velocity.getX() * 0.02),
                currentPose.getY() + (velocity.getY() * 0.02),
                currentPose.getRotation());

        double deltaY = vision.getSpeakerPos().getY() - newPose.getY();
        double angle = mult * -Math.asin(deltaY / vision.getDistancetoSpeaker(newPose));

        double deltaAngle = angle - previousAngle;
        double omega = deltaAngle / 0.02;

        double driveHeading = drive.getRotationHeading().getRadians();
        double output = rotationPID.calculate(driveHeading, angle);

        double x = (DriverStation.isTeleop()) ? MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband) : 0;
        double y = (DriverStation.isTeleop()) ? MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband) : 0;

        drive.runVelocity(new ChassisSpeeds(
                -Math.copySign(x * x, x) * DriveConstants.kMaxSpeedMetersPerSecond,
                -Math.copySign(y * y, y) * DriveConstants.kMaxSpeedMetersPerSecond,
                MathUtil.clamp(output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed)), 
                true);

        Logger.recordOutput("Vision/drive heading", driveHeading);
        Logger.recordOutput("Vision/Speaker Position", vision.getSpeakerPos());
        Logger.recordOutput("Vision/angle", angle);
        Logger.recordOutput("Vision/output", output);
        Logger.recordOutput("Vision/NewPose", newPose);
        Logger.recordOutput("Vision/previous angle", previousAngle);
        // Logger.recordOutput("Vision/Setpoint angle", rotationPID.getSetpoint().position);

        Logger.recordOutput("Vision/omega", omega);

        previousAngle = angle;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
