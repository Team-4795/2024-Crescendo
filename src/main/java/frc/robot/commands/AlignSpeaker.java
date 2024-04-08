package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;

public class AlignSpeaker extends Command {

    private Drive drive = Drive.getInstance();

    public AlignSpeaker() {
        addRequirements(drive);
    }

    @Override
    public void initialize(){
        AlignPose.setState(AlignPose.State.SPEAKER);
    }

    @Override
    public void execute() {
        double x = (DriverStation.isTeleop()) ? MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband) : 0;
        double y = (DriverStation.isTeleop()) ? MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband) : 0;
        double output = AlignPose.calculateRotationSpeed();

        drive.runVelocity(new ChassisSpeeds(
                -Math.copySign(x * x, x) * DriveConstants.kMaxSpeedMetersPerSecond,
                -Math.copySign(y * y, y) * DriveConstants.kMaxSpeedMetersPerSecond,
                MathUtil.clamp(output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed)), 
                true);
    }

    @Override
    public void end(boolean interrupted){
        if(DriverStation.isAutonomous()){
            drive.runVelocity(new ChassisSpeeds(0, 0, 0), false);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
