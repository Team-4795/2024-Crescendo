package frc.robot.commands;

import org.littletonrobotics.junction.Logger    ;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.AprilTagVision.Vision;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class AlignToGamepiece extends Command {
    private Drive drive;
    private IntakeCamVision vision = IntakeCamVision.getInstance();

    private PIDController rotationPID = new PIDController(0.35, 0, 0); 

    private Pose2d BLUE_SOURCE = new Pose2d(15.8,0.8, Rotation2d.fromDegrees(-45));
    private Pose2d RED_SOURCE = new Pose2d(0.8,0.5, Rotation2d.fromDegrees(45));
    private Pose2d sourcePose;
    
    public AlignToGamepiece(Drive drive) {
        addRequirements(drive);
        if(Constants.hasVision){
            vision = IntakeCamVision.getInstance();
            addRequirements(vision);
        }
        rotationPID.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize(){  
            DriverStation.getAlliance().ifPresent((alliance) -> {
            sourcePose = (alliance == Alliance.Blue) ? BLUE_SOURCE : RED_SOURCE;
        });
        
    }

    @Override
    public void execute() {
       double lifecamYaw = vision.getIntakeCamYaw();
       boolean hasTargets = vision.intakeCamHasTargets();

       double distanceToSource = drive.getPose().getTranslation().getDistance(sourcePose.getTranslation());
       double driveHeading = Units.degreesToRadians(drive.getWrappedHeading());
       
       double x = MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband);
       double y = MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband);
       double output = -MathUtil.applyDeadband(OIConstants.driverController.getRightX(), OIConstants.kAxisDeadband);

       if (hasTargets) {
            output = MathUtil.clamp(rotationPID.calculate(Units.degreesToRadians(lifecamYaw), 0), -1, 1);
       } 
       else if (!hasTargets && distanceToSource < 7) {
            output = MathUtil.clamp(rotationPID.calculate(driveHeading, sourcePose.getRotation().getRadians()), -1, 1);
       }

       drive.drive(
                -Math.copySign(x * x, x),
                -Math.copySign(y * y, y),
                output,
                false, true);   
        
        Logger.recordOutput("Vision/Note Yaw", lifecamYaw);
        Logger.recordOutput("Vision/Note PID", output);
        Logger.recordOutput("Vision/drive heading", driveHeading);
        Logger.recordOutput("Vision/Source Pos", sourcePose);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
