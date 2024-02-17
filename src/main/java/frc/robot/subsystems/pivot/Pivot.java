package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.util.LoggedTunableNumber;

public class Pivot extends SubsystemBase {
    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", PivotConstants.kP);
    LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI", PivotConstants.kI);
    LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", PivotConstants.kD);

    LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", PivotConstants.kV);
    LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/kA", PivotConstants.kA);
    LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", PivotConstants.kS);

    private ProfiledPIDController controller = new ProfiledPIDController(
            kP.get(), kI.get(), kD.get(),
            PivotConstants.constraints);

    private SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(
            kS.get(), kV.get(), kA.get());

    private double goal = 0;
    private final boolean disableArm = false;
    private boolean idleMode = true;
    private double manualVolts = 0; //temporary, will change to setting setpoint instead later

    PivotVisualizer visualizer = new PivotVisualizer(Color.kDarkOrange);

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

        visualizer.update(360 * getPosition() / (2 * Math.PI));
        controller.setTolerance(Units.degreesToRadians(0.5));

        setDefaultCommand(run(() -> {
            double up = MathUtil.applyDeadband(
                    OIConstants.driverController.getRightTriggerAxis(), OIConstants.kAxisDeadband);
            double down = MathUtil.applyDeadband(
                    OIConstants.driverController.getLeftTriggerAxis(), OIConstants.kAxisDeadband);
            
            manualVolts = 0.15 * (Math.pow(up, 3) - Math.pow(down, 3)) * 12;
            // double change = PivotConstants.manualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));
            
            // double change = -PivotConstants.manualSpeed *
            // MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(),
            // OIConstants.kAxisDeadband);

            // setGoal(goal + change);
        }));
    }

    public void setGoal(double goal) {
        this.goal = MathUtil.clamp(goal, PivotConstants.lowLimit, PivotConstants.highLimit);
        controller.setGoal(goal);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        visualizer.update(Units.radiansToDegrees(getPosition() + PivotConstants.angleOffset));

        LoggedTunableNumber.ifChanged(hashCode(), () -> controller.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> motorFeedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get()), kS, kV, kA);
                
        double PIDVolts = controller.calculate(getPosition());  
        double FFVolts = motorFeedforward.calculate(controller.getSetpoint().velocity);

        if (!disableArm) {
            io.setVoltage(manualVolts + FFVolts + linearFF(getPosition()));
        }


        Logger.recordOutput("Pivot/PID Volts", PIDVolts);
        Logger.recordOutput("Pivot/FF Volts", FFVolts);
        Logger.recordOutput("Pivot/Setpoint Position", controller.getSetpoint().position);
        Logger.recordOutput("Pivot/Setpoint Velocity", controller.getSetpoint().velocity);
        Logger.recordOutput("Pivot/Goal", goal);

    }
    
    public void toggleIdleMode() {
        idleMode = !idleMode;
        io.setIdleMode(idleMode);
    }

    public double torqueFromAngle(double angleRad) {
        double springAngle = Math.atan2(
                PivotConstants.d * Math.sin(angleRad) + PivotConstants.y,
                -PivotConstants.d * Math.cos(angleRad) + PivotConstants.x);
        double Tg = -PivotConstants.M * PivotConstants.R * PivotConstants.g * Math.cos(angleRad);
        double Ts = PivotConstants.d * PivotConstants.F * Math.sin(springAngle - (Math.PI - angleRad));
        return Tg + Ts;
    }

    public double linearFF(double angle) {
        return -0.24 * angle - 0.01;
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
        return inputs.pivotMotorPositionRads;
    }

    public double getTruePosition() {
        return inputs.pivotMotorPositionRads + PivotConstants.angleOffset;
    }

    public double getVelocity() {
        return inputs.pivotMotorVelocityRadPerSec;
    }

    public void reset() {
        controller.reset(getPosition());
        this.setGoal(getPosition());
    }
}
