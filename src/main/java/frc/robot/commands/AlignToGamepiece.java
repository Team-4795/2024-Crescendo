package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.MAXSwerve.DriveConstants;
import frc.robot.subsystems.vision.intakeCam.IntakeCamVision;

public class AlignToGamepiece extends Command {
    private Drive drive = Drive.getInstance();
    private IntakeCamVision vision = IntakeCamVision.getInstance();

    private PIDController rotationPID = new PIDController(0.35, 0, 0);
    private boolean hasTargets;
    private boolean startTimer;
    private double time;

    public AlignToGamepiece() {
        vision = IntakeCamVision.getInstance();
        rotationPID.enableContinuousInput(-180, 180);
        rotationPID.setTolerance(3);
        addRequirements(vision, drive);
    }

    @Override
    public void initialize() {
        AlignPose.setState(AlignPose.State.SOURCE);
        rotationPID.reset();
    }

    @Override
    public void execute() {
        double lifecamYaw = vision.getIntakeCamYaw();

        double x = MathUtil.applyDeadband(OIConstants.driverController.getLeftY(), OIConstants.kAxisDeadband);
        double y = MathUtil.applyDeadband(OIConstants.driverController.getLeftX(), OIConstants.kAxisDeadband);
        double output = AlignPose.calculateRotationSpeed();

        if (vision.intakeCamHasTargets()) {
            hasTargets = true;
            startTimer = true;
        }

        if (!vision.intakeCamHasTargets() && startTimer) {
            startTimer = false;
            time = Timer.getFPGATimestamp();
        }

        if(time >= 0.5){
            hasTargets = false;
        }

        if(hasTargets){
            output = rotationPID.calculate(lifecamYaw, 0) * DriveConstants.kMaxAngularSpeed;
        }

        output = MathUtil.clamp(output, -DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed);

        drive.runVelocity(new ChassisSpeeds(
                -Math.copySign(x * x, x) * DriveConstants.kMaxSpeedMetersPerSecond,
                -Math.copySign(y * y, y) * DriveConstants.kMaxSpeedMetersPerSecond,
                output), true);

        Logger.recordOutput("Vision/Note Yaw", lifecamYaw);
        Logger.recordOutput("Vision/Note PID", output);
        Logger.recordOutput("Vision/HasTargets", hasTargets);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
