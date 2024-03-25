package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerSetpoints;
import frc.robot.Constants.PivotSetpoints;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;

public class AutoAlignAmp extends Command{
    private static final Pose2d RED_AMP = new Pose2d(14.7, 7.6, Rotation2d.fromRadians(Math.PI / 2));
    private static final Pose2d BLUE_AMP = new Pose2d(1.86, 7.6, Rotation2d.fromRadians(Math.PI / 2));

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private double mult;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double distance;

    private Drive drive;
    private Pivot pivot;
    private Shooter shooter;
    private Indexer indexer;

    public AutoAlignAmp(ProfiledPIDController translation, ProfiledPIDController rotation) {
        translationController = translation;
        translationController.setTolerance(0.1);
        rotationController = rotation;
        drive = Drive.getInstance();
        pivot = Pivot.getInstance();
        shooter = Shooter.getInstance();
        indexer = Indexer.getInstance();
        addRequirements(drive, pivot, shooter);
    }


    @Override
    public void initialize(){
        DriverStation.getAlliance().ifPresent((alliance) -> {
            targetPose = (alliance == Alliance.Blue) ? BLUE_AMP : RED_AMP;
            mult = (alliance == Alliance.Red) ? -1.0 : 1.0;
        });
        currentPose = Drive.getInstance().getPose();
        double velocity = mult * projection(drive.getFieldRelativeTranslationVelocity(), targetPose.getTranslation().minus(currentPose.getTranslation()));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        Logger.recordOutput("AutoAlign/Robot velocity", drive.getFieldRelativeTranslationVelocity());
        Logger.recordOutput("AutoAlign/Translation", targetPose.getTranslation().minus(currentPose.getTranslation()));
        Logger.recordOutput("AutoAlign/velocity", velocity);
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(distance, velocity);
        rotationController.reset(MathUtil.angleModulus(currentPose.getRotation().getRadians()), drive.getTurnRate());
    }

    @Override
    public void execute() {
        currentPose = Drive.getInstance().getPose();
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        translationController.reset(distance, translationController.getSetpoint().velocity);

        double rotationPIDOutput = rotationController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetPose.getRotation().getRadians());
        double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;
        
        double scalar = scalar(distance);
        double drivePIDOutput = translationController.calculate(distance, 0);
        double driveSpeed = mult * scalar * translationController.getSetpoint().velocity + drivePIDOutput;
        Rotation2d direction = new Rotation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());
        // Rotation2d direction = new Rotation2d(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());

        drive.runVelocity(new ChassisSpeeds(driveSpeed * direction.getCos(), driveSpeed * direction.getSin(), omega));

        if(distance < 1.0){
                pivot.setGoal(PivotSetpoints.amp);
                shooter.setShootingSpeedRPM(ShooterSetpoints.ampTop, ShooterSetpoints.ampBottom);
        }
        if(rotationController.atGoal() && distance < 0.1){
                indexer.setIndexerSpeed(IndexerSetpoints.shoot);
        }

        Logger.recordOutput("AutoAlign/target pose", targetPose);
        Logger.recordOutput("AutoAlign/Translation x direction", driveSpeed * direction.getCos());
        Logger.recordOutput("AutoAlign/Translation y direction", driveSpeed * direction.getSin());
        Logger.recordOutput("AutoAlign/Rotation setpoint position", rotationController.getSetpoint().position);
        Logger.recordOutput("AutoAlign/Rotation setpoint velocity", rotationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/Rotation", MathUtil.angleModulus(currentPose.getRotation().getRadians()));
        Logger.recordOutput("AutoAlign/Rotation at goal", rotationController.atGoal());

        Logger.recordOutput("AutoAlign/Translation setpoint position", translationController.getSetpoint().position);
        Logger.recordOutput("AutoAlign/Translation setpoint velocity", translationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/Distance", currentPose.getTranslation().getDistance(targetPose.getTranslation()));
        Logger.recordOutput("AutoAlign/Distance at goal", translationController.atGoal());
        Logger.recordOutput("AutoAlign/PID input", drivePIDOutput);
    }

    @Override
    public void end(boolean interuppted){
        indexer.setIndexerSpeed(0);
        pivot.setGoal(PivotSetpoints.stow);
        shooter.setShootingSpeedRPM(0, 0);
    }

    private double projection(Translation2d v1, Translation2d onto){
        Vector<N2> velocity = VecBuilder.fill(v1.getX(), v1.getY());
        Vector<N2> translation = VecBuilder.fill(onto.getX(), onto.getY());
        Vector<N2> projection = velocity.projection(translation);
        if(projection.dot(translation) > 0) {
              return -Math.sqrt(projection.dot(projection));  
        } else {
           return Math.sqrt(projection.dot(projection));     
        }
    }

    private double scalar(double distance){
        return 1 / (1 + Math.exp(-6.5 * (distance - 0.6)));
    }
}
