package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotSetpoints;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.pivot.PivotConstants.PivotMode;
import frc.robot.subsystems.vision.AprilTagVision.Vision;
import frc.robot.util.LoggedProfiledPIDController;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;

public class Pivot extends SubsystemBase {


    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    @AutoLogOutput
    private PivotMode mode = PivotMode.FAST;

    LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", PivotConstants.kP);
    LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI", PivotConstants.kI);
    LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", PivotConstants.kD);

    LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", PivotConstants.kV);
    LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/kA", PivotConstants.kA);
    LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", PivotConstants.kS);

    LoggedTunableNumber stabilizer = new LoggedTunableNumber("Pivot/Accel Stability", 0.02);

    LoggedTunableNumber encoderDistance = new LoggedTunableNumber("Pivot/Rel. Encoder Interpolation Distance", 0.4);
    LoggedTunableNumber absEncoderDistance = new LoggedTunableNumber("Pivot/Abs. Encoder Interpolation Distance", 0.0);

    LoggedTunableNumber regressionSetpoint = new LoggedTunableNumber("Pivot/Regression setpoint", 0.5);

    private SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(
            kS.get(), kV.get(), kA.get());

    private LoggedProfiledPIDController controller = new LoggedProfiledPIDController(
        PivotConstants.kP,
        PivotConstants.kI, 
        PivotConstants.kD,
        PivotConstants.constraints);

    @AutoLogOutput
    private double lastGoal = 0;

    @AutoLogOutput
    private double goal = 0;
    private final boolean disableArm = false;
    private boolean idleMode = true;

    private double maxDistance = 0.1;
    private double minDistance = 0.0;
    private double distance = 0.0;

    private SysIdRoutine sysid;

    PivotVisualizer visualizer = new PivotVisualizer();

    private static Pivot instance;

    public static Pivot getInstance() {
        return instance;
    }

    public static Pivot initialize(PivotIO io) {
        if (instance == null) {
            instance = new Pivot(io);
        }
        return instance;
    }

    public Pivot(PivotIO pivotIO) {
        io = pivotIO;
        io.updateInputs(inputs);

        controller.setIntegratorRange(-0.02, 0.02);

        visualizer.update(360 * getTruePosition() / (Math.PI * 2), Units.radiansToDegrees(controller.getSetpoint().position + PivotConstants.angleOffset));

        sysid = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.2).per(Seconds.of(1)), Volts.of(2), null, (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> {
                runVoltage(voltage.in(Volts));
            }, null, this));

        setDefaultCommand(run(() -> {
            double up = MathUtil.applyDeadband(
                    OIConstants.operatorController.getRightTriggerAxis(), OIConstants.kAxisDeadband);
            double down = MathUtil.applyDeadband(
                    OIConstants.operatorController.getLeftTriggerAxis(), OIConstants.kAxisDeadband);
            
            double change = PivotConstants.manualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));
            // double change = PivotConstants.manualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));
            
            // double change = -PivotConstants.manualSpeed *
            // MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(),
            // OIConstants.kAxisDeadband);
            if(DriverStation.isTeleop()){
                setGoal(goal + change);   
            }
        }));
    }

    public Pose3d getPose() {
        var drivePose = Drive.getInstance().getPose();
        // return new Pose3d(drivePose.getY(), 0.254 + drivePose.getX(), 0.215, new Rotation3d((0.62)-Units.degreesToRadians(getTruePosition()), 0, 0)).rotateBy(new Rotation3d(0, 0, -Math.PI / 2));
        var pose1 = new Pose3d(0.254 + Math.sin(getTruePosition()) * 0.112, 0, 0.215 + Math.cos(getTruePosition()) * 0.112, new Rotation3d(0, -getTruePosition(), 0))
            .rotateBy(new Rotation3d(0, 0, drivePose.getRotation().getRadians() + Math.PI));

        return new Pose3d(-pose1.getX() + drivePose.getX(), -pose1.getY() + drivePose.getY(), pose1.getZ(), pose1.getRotation()).transformBy(new Transform3d(0.35, 0, 0, new Rotation3d()));
    }

    public void setGoal(double goal) {
        if (goal != this.goal) {
            // this.lastGoal = this.goal;
            this.goal = MathUtil.clamp(goal, PivotConstants.lowLimit, PivotConstants.highLimit);
            io.resetEncoders();
            if(Math.abs(goal - this.getPosition()) < 0.4){
                setPivotMode(PivotMode.SLOW);
            } else {
                setPivotMode(PivotMode.FAST);
            }
        }
    }

    private void setPivotMode(PivotMode mode){
        this.mode = mode;
        controller.setPID(mode.getSettings().kP(), mode.getSettings().kI(), mode.getSettings().kD());
        controller.setConstraints(mode.getSettings().constraints());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }

    // public Command aim(){
    //     switch(StateManager.getState()){
    //         case AMP:
    //             return this.aimAmp();
    //         case SPEAKER:
    //             return this.aimSpeakerDynamic();
    //         case SHUTTLE:
    //             return this.aimShuttle();
    //         default:
    //             return null;
    //     }
    // } 

    public Command aimSpeakerDynamic(){
        return Commands.either(
            Commands.run(() -> {
                if(Constants.tuningMode){
                    this.setGoal(regressionSetpoint.get());
                } else {
                    double distanceToSpeaker = Vision.getInstance().getDistancetoSpeaker(Drive.getInstance().getPose());
                    this.setGoal(PivotConstants.armAngleMap.get(distanceToSpeaker));
                }
            }).finallyDo(() -> setGoal(PivotSetpoints.stow)), 
            Commands.startEnd(
                () -> setGoal(PivotSetpoints.speaker),
                () -> setGoal(PivotSetpoints.stow)),
            () -> StateManager.isAutomate());
    }

    public Command aimShuttle(){
        return Commands.startEnd(
            () -> setGoal(PivotSetpoints.shuttle), 
            () -> setGoal(PivotSetpoints.stow)
        ).alongWith(aiming());
    }

    public Command aimAmp() {
        return Commands.startEnd(
            () -> setGoal(PivotSetpoints.amp),
            () -> setGoal(PivotSetpoints.stow)
        ).alongWith(aiming());
    }

    public Command aimAmpManual(){
        return Commands.startEnd(
            () -> setGoal(PivotSetpoints.manualAmp),
            () -> setGoal(PivotSetpoints.stow)
        ).alongWith(aiming());
    }

    public Command aimSource() {
        return Commands.startEnd(
            () -> setGoal(PivotSetpoints.source),
            () -> setGoal(PivotSetpoints.stow)
        ).alongWith(aiming());
    }

    public Command aimIntake() {
        return Commands.startEnd(
            () -> setGoal(PivotSetpoints.intake),
            () -> setGoal(PivotSetpoints.stow)
        );
    }

    public Command aiming() {
        return Commands.startEnd(() -> StateManager.setAim(true), () -> StateManager.setAim(false));
    }

    @AutoLogOutput
    public boolean atGoal() {
        return Util.epsilonEquals(goal, getPosition(), Tolerances.pivotSetpoint);
    }

    public void setScheduledP() {
        double distance = Math.abs(goal - controller.getSetpoint().position);
        double x = MathUtil.clamp((0.4 - distance) / 0.4, 0, 1);
        controller.setPID(x * kP.get(), kI.get(), kD.get());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        visualizer.update(Units.radiansToDegrees(getTruePosition()), Units.radiansToDegrees(controller.getSetpoint().position + PivotConstants.angleOffset));

        setScheduledP();
        LoggedTunableNumber.ifChanged(hashCode(), () -> motorFeedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get()), kS, kV, kA);

        double v1 = controller.getSetpoint().velocity;
        double PIDVolts = controller.calculate(getPosition(), goal);
        double FFVolts = Constants.useLQR ? kS.get() * Math.signum(controller.getSetpoint().velocity) : motorFeedforward.calculate(v1, controller.getSetpoint().velocity, 0.02);
        // linearFF(getPosition()) + getAccelerationCompensation())

        if (!disableArm) {
            if (DriverStation.isDisabled()) {
                io.setVoltage(0);
            } else {
                if(mode == PivotMode.FAST){
                    io.setVoltage(PIDVolts + FFVolts + linearFF() + getAccelerationCompensation());
                } else {
                    io.setVoltage(PIDVolts + linearFF() + getAccelerationCompensation());
                }
            }
        }

        Logger.recordOutput("Pivot/Position", getPosition());
        Logger.recordOutput("Pivot/Stabilization Compensation Voltage", getAccelerationCompensation());
        Logger.recordOutput("Pivot/PID Volts", PIDVolts);
        Logger.recordOutput("Pivot/FF Volts", FFVolts);
        Logger.recordOutput("Pivot/Static gain volts", linearFF());
        Logger.recordOutput("Pivot/Setpoint Position", controller.getSetpoint().position);
        Logger.recordOutput("Pivot/Setpoint Velocity", controller.getSetpoint().velocity);
        Logger.recordOutput("Pivot/Goal", goal);
        Logger.recordOutput("Pivot/Total Error", controller.getTotalError());
        Logger.recordOutput("Pivot/P Volts", controller.getPVolts());
        Logger.recordOutput("Pivot/I Volts", controller.getIVolts());
        Logger.recordOutput("Pivot/D Volts", controller.getDVolts());
    }
    
    public void toggleIdleMode() {
        idleMode = !idleMode;
        io.setIdleMode(idleMode);
    }

    // public double torqueFromAngle(double angleRad) {
    //     double springAngle = Math.atan2(
    //             PivotConstants.d * Math.sin(angleRad) + PivotConstants.y,
    //             -PivotConstants.d * Math.cos(angleRad) + PivotConstants.x);
    //     double Tg = -PivotConstants.M * PivotConstants.R * PivotConstants.g * Math.cos(angleRad);
    //     double Ts = PivotConstants.d * PivotConstants.F * Math.sin(springAngle - (Math.PI - angleRad));
    //     return Tg + Ts;
    // }

    public double linearFF() {
        return 0.22 * Math.cos(getPosition() + 1.62);
    }

    public void runVoltage(double volts) {
        if (disableArm) {
            io.setVoltage(volts);
        } else {
            throw new IllegalArgumentException("Setting direct pivot voltage without disabling arm!");
        }
    }

    // Choose between motor position or absolute encoder position 
    public double getPosition() {
        distance = Math.abs(goal - controller.getSetpoint().position);
        double x = (1.0 / (encoderDistance.get() - absEncoderDistance.get())) * (distance - absEncoderDistance.get());
        x = MathUtil.clamp(x, 0, 1);
        return inputs.pivotPositionRads * (1 - x) + inputs.pivotMotorPositionRads * x;
    }

    public double getAccelerationCompensation(){
        // var driveAccel = Drive.getInstance().getIMUAcceleration();
        // Rotation2d pivotHeading = Drive.getInstance().getRotationHeading().plus(Rotation2d.fromDegrees(180));
        // Vector<N2> pivotVector = VecBuilder.fill(pivotHeading.getCos(), pivotHeading.getSin());
        // var pivotAccel = driveAccel.projection(pivotVector);
        // double scalar = (pivotAccel.dot(pivotVector) >= 0) ? -1.0 : 1.0;
        // double accel = pivotAccel.norm() * Math.sin(getTruePosition());
        return stabilizer.get() * Drive.getInstance().getXAccel();
    }

    public double getTruePosition() {
        return getPosition() + PivotConstants.angleOffset;
    }

    public double getVelocity() {
        return inputs.pivotMotorVelocityRadPerSec;
    }

    public void reset() {
        controller.reset(getPosition());
        io.resetEncoders();
        this.setGoal(getPosition());
    }

    private double scalar(){
        if(distance > maxDistance){
            return 1.0;
        } else if (minDistance < distance && distance < maxDistance){
            return MathUtil.clamp((1 / (maxDistance - minDistance)) * (distance - minDistance), 0, 1);
        } else {
            return 0.0;
        }
    }
}
