package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private PIDController rotationPID = new PIDController(0.09, 0, 0); // 0.09, 0, 0

    public AlignSpeaker() {
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        DriverStation.getAlliance().ifPresent((alliance) -> {
            if(alliance == Alliance.Blue){
                AlignPose.setTarget(FieldConstants.BLUE_SPEAKER, true, Alliance.Blue);
            } else {
                AlignPose.setTarget(FieldConstants.RED_SPEAKER, true, Alliance.Red);
            }
        });
    }

    @Override
    public void execute() {

        double x = (DriverStation.isTeleop()) ? MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband) : 0;
        double y = (DriverStation.isTeleop()) ? MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband) : 0;
        double output = AlignPose.calculateRotationSpeed();

        drive.runVelocity(new ChassisSpeeds(
                -Math.copySign(x * x, x) * DriveConstants.kMaxSpeedMetersPerSecond,
                -Math.copySign(y * y, y) * DriveConstants.kMaxSpeedMetersPerSecond,
                output));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
