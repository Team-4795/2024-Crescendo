package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerSetpoints;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotSetpoints;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;

public class AlignShuttle extends Command {

    private Drive drive = Drive.getInstance();
    private Pivot pivot = Pivot.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Indexer indexer = Indexer.getInstance();

    private double velocityCutoff = 2.0;
    private double shootingSpeed;
    private Alliance alliance;

    public AlignShuttle(){
        addRequirements(drive, pivot, shooter);
    }

    @Override
    public void initialize() {
        DriverStation.getAlliance().ifPresent((alliance) -> {
            this.alliance = alliance;
        });
        AlignPose.setState(AlignPose.State.SHUTTLE);
        pivot.setGoal(PivotSetpoints.shuttle);
    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband);
        double y = MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband);
        double output = AlignPose.calculateRotationSpeed();

        drive.runVelocity(new ChassisSpeeds(
                -Math.copySign(x * x, x) * DriveConstants.kMaxSpeedMetersPerSecond,
                -Math.copySign(y * y, y) * DriveConstants.kMaxSpeedMetersPerSecond,
                output),
                true);

                
        // double shootShuttle = OIConstants.operatorController.getRightY() * 1000 + 3000;
               
        shootingSpeed = ShooterConstants.shuttleSpeeds.get(AlignPose.getDistanceToTarget()); //always positive
        shooter.setShootingSpeedRPM(-shootingSpeed, shootingSpeed);
        Logger.recordOutput("AlignShuttle/shootingSpeed", shootingSpeed);

        if(alliance == Alliance.Blue){
            if(drive.getPose().getX() < FieldConstants.RED_WING_X && drive.getTranslationVelocity().getNorm() < velocityCutoff && AlignPose.atGoal() && shooter.atGoal()){
                indexer.setIndexerSpeed(IndexerSetpoints.shoot);
            }
        } else {
            if(drive.getPose().getX() > FieldConstants.BLUE_WING_X && drive.getTranslationVelocity().getNorm() < velocityCutoff && AlignPose.atGoal() && shooter.atGoal()){
                indexer.setIndexerSpeed(IndexerSetpoints.shoot);
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        pivot.setGoal(PivotSetpoints.stow);
        shooter.setShootingSpeedRPM(0, 0);
        indexer.setIndexerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
     
    }
}
