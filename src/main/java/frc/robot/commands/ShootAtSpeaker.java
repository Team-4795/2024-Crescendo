package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;

public class ShootAtSpeaker extends Command {
    private Vision vision;
    private static ProfiledPIDController rotationPID = new ProfiledPIDController(12, 0, 0, new Constraints(10, 8)); // Change Values

    private final Drive drive = Drive.getInstance();
    private final Pivot pivot = Pivot.getInstance(); 
    private final LEDs leds = LEDs.getInstance();

    private static final double shootingSpeed = 17;

    private record AimData(double time, Translation2d noteVel, Rotation2d heading, double pivotAngle) {}

    private Rotation2d previousAngle = new Rotation2d();

    public ShootAtSpeaker() {
        addRequirements(drive, pivot, leds);

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(5));

        if(Constants.hasVision){
            vision = Vision.getInstance();
            addRequirements(vision);
        }
    }

    @Override
    public void initialize() {
        Pose2d robotPose = drive.getPose();
        Translation2d velocity = drive.getTranslationVelocity().rotateBy(drive.getRotationHeading());
        rotationPID.reset(drive.getRotationHeading().getRadians() + Math.PI, drive.getTurnRate());
        Pose2d nextPose = robotPose.plus(new Transform2d(velocity.times(0.02), new Rotation2d()));
        AimData results = shootingRotation(nextPose);
        previousAngle = results.heading;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drive.getPose();
        
        Translation2d velocity = drive.getTranslationVelocity().rotateBy(drive.getRotationHeading());
        Pose2d nextPose = robotPose.plus(new Transform2d(velocity.times(0.02), new Rotation2d()));

        AimData results = shootingRotation(nextPose);

        // Pose2d collisionPos = new Pose2d(nextPose.getX() + (results[1] * results[0]),
        //                                 nextPose.getY() + (results[2] * results[0]), 
        //                                 nextPose.getRotation());

        Rotation2d driveHeading = drive.getRotationHeading().plus(new Rotation2d(Math.PI));

        double pid = rotationPID.calculate(driveHeading.getRadians(), results.heading.getRadians());
        double ff = results.heading.minus(previousAngle).getRadians() / 0.02;
        // double ff = rotationPID.getSetpoint().velocity;

        drive.runVelocity(new ChassisSpeeds(
            -MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
            -MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband) * DriveConstants.kMaxSpeedMetersPerSecond,
            MathUtil.clamp(ff + pid, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed)
        ));

        pivot.setGoal(results.pivotAngle - PivotConstants.angleOffset);

        if (pivot.atSetpoint() && Shooter.getInstance().atSetpoint() && rotationPID.atSetpoint()) {
            leds.setColor(Color.kMagenta);
        } else {
            leds.setColor(Color.kFirstBlue);
        }

        previousAngle = results.heading;

        // Logger.recordOutput("Vision/Collision pos", collisionPos);
        // Logger.recordOutput("Vision/omega", output);
        Logger.recordOutput("Vision/Setpoint", rotationPID.getSetpoint().position);
        Logger.recordOutput("Vision/Altered angle", results.heading.getRadians());
        Logger.recordOutput("Vision/Current angle", driveHeading.getRadians());
    }

    // https://stackoverflow.com/questions/17204513/how-to-find-the-interception-coordinates-of-a-moving-target-in-3d-space
    // Returns [time, vel x, vel y, robot rotation, pivot angle]
    private AimData shootingRotation(Pose2d pose) {
        Translation3d speakerPose = vision.getSpeakerPos();

        Translation3d armpos = new Pose3d(pose.getX(), pose.getY(), 0.0, new Rotation3d(0, 0, pose.getRotation().getRadians()))
            .transformBy(new Transform3d(0.254, 0, 0.25, new Rotation3d()))
            .getTranslation();

        // X 0.23 Z 0.58
        Vector<N3> speakerPos = VecBuilder.fill(speakerPose.getX(), speakerPose.getY(), speakerPose.getZ());
        Vector<N3> robotPos = VecBuilder.fill(armpos.getX(), armpos.getY(), armpos.getZ());

        Translation2d vel = drive.getTranslationVelocity().rotateBy(pose.getRotation()).unaryMinus();
        Vector<N3> velVector = VecBuilder.fill(vel.getX(), vel.getY(), 0);

        double a = velVector.dot(velVector) - Math.pow(shootingSpeed, 2);
        double b = 2 * (speakerPos.dot(velVector) - robotPos.dot(velVector));
        double c = speakerPos.dot(speakerPos) + robotPos.dot(robotPos) - 2 * robotPos.dot(speakerPos);

        double time1 = (-b + Math.sqrt((b*b) - 4*a*c)) / (2*a);
        double time2 = (-b - Math.sqrt((b*b) - 4*a*c)) / (2*a);

        double t;

        if (Double.isNaN(time1) || time2 > 0.0) {
            t = time2;
        } else if (Double.isNaN(time2) || time1 > 0.0) {
            t = time1;
        } else {
            t = Math.min(time1, time2);
        }

        Vector<N3> output = (speakerPos.minus(robotPos).plus(velVector.times(t))).div(t * shootingSpeed);

        // double robotAngle = new Rotation2d(output.get(0, 0), output.get(1, 0)).getRadians();

        double pivotAngle = Math.asin(output.get(2, 0) / output.norm());
        double angle = Math.atan2(output.get(1, 0), output.get(0, 0));

        // Logger.recordOutput("Vision/Pivot angle", pivotAngle);

        // Logger.recordOutput("Vision/Arm", new Pose3d[] {armpos});

        // Logger.recordOutput("Vision/Angle 2", new Rotation3d(output).getZ());
        // Logger.recordOutput("Vision/Angle 3", new Rotation3d(output).getY());
        // Logger.recordOutput("Vision/Angle 4", new Rotation3d(output).getX());

        // double pivotAngle = Math.asin(output.get(2, 0))
        return new AimData(t, new Translation2d(output.get(0, 0) * shootingSpeed, output.get(1, 0) * shootingSpeed), new Rotation2d(angle), pivotAngle);
    }
}