package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;
import frc.robot.util.LoggedTunableNumber;

public class AlignToGamepiece extends Command {
    private Drive drive = Drive.getInstance();
    private IntakeCamVision vision = IntakeCamVision.getInstance();

    private LoggedTunableNumber kP = new LoggedTunableNumber("Gamepiece Align/kP", 1.35);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Gamepiece Align/kD", 0.1);

    private PIDController rotationPID = new PIDController(kP.get(), 0, kD.get());
    private boolean hasTargets;
    private boolean startTimer;
    private double time;
    private boolean fieldRelative = true;
    private Translation2d sourcePose;

    public AlignToGamepiece() {
        vision = IntakeCamVision.getInstance();
        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(3);
        addRequirements(vision, drive);
    }

    @Override
    public void initialize() {
        DriverStation.getAlliance().ifPresent((alliance) -> {
            if(alliance == Alliance.Blue){
                sourcePose = FieldConstants.BLUE_SOURCE.getTranslation();
            } else {
                sourcePose = FieldConstants.RED_SOURCE.getTranslation();
            }
        });
        AlignPose.setState(AlignPose.State.SOURCE);
        rotationPID.reset();
    }

    @Override
    public void execute() {
        // rotationPID.setPID(kP.get(), 0, kD.get());
        double lifecamYaw = Units.degreesToRadians(vision.getIntakeCamYaw());

        double x = MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband);
        double y = MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband);
        double output = -MathUtil.applyDeadband(OIConstants.driverController.getRightX(), OIConstants.kAxisDeadband);
        fieldRelative = true;
        
        if(Drive.getInstance().getPose().getTranslation().getDistance(sourcePose) < 10){
            output = AlignPose.calculateRotationSpeed();
        }

        hasTargets = vision.intakeCamHasTargets();

        if(hasTargets){
            output = rotationPID.calculate(lifecamYaw, 0) * DriveConstants.kMaxAngularSpeed;
            fieldRelative = false;
            OIConstants.driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
        }

        output = MathUtil.clamp(output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed);

        drive.runVelocity(new ChassisSpeeds(
                -Math.copySign(x * x, x) * DriveConstants.kMaxSpeedMetersPerSecond,
                -Math.copySign(y * y, y) * DriveConstants.kMaxSpeedMetersPerSecond,
                output), fieldRelative);

        Logger.recordOutput("Vision/Note Yaw", lifecamYaw);
        Logger.recordOutput("Vision/Note PID", output);
        Logger.recordOutput("Vision/HasTargets", hasTargets);
    }

    @Override
    public void end(boolean interrupted){
        OIConstants.driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
