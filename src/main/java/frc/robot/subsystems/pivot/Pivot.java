package frc.robot.subsystems.pivot;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

public class Pivot extends SubsystemBase {
    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private ProfiledPIDController controller = 
         new ProfiledPIDController(PivotConstants.kP, PivotConstants.KI, PivotConstants.kD, new Constraints(2, 2));
  
    private final ArmFeedforward pivotFeedForward = 
         new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);
    private double goal = 0;

    PivotVisualizer visualizer;

    private static Pivot instance;

    public static Pivot getInstance(){
        return instance;
    }

    public static Pivot initialize(PivotIO io){
        if(instance == null){
            instance = new Pivot(io);
        }
        return instance;
    }


    public Pivot(PivotIO pivotIO) {
        io = pivotIO;
        visualizer = new PivotVisualizer(Color.kDarkOrange);
        io.updateInputs(inputs);
        goal = inputs.pivotPositionRads;
        visualizer.update(360 * inputs.pivotPositionRads);

        setDefaultCommand(run(() -> {
            double up = MathUtil.applyDeadband(
                    OIConstants.operatorController.getRightTriggerAxis(), PivotConstants.deadband);
            double down = MathUtil.applyDeadband(
                    OIConstants.operatorController.getLeftTriggerAxis(), PivotConstants.deadband);
            double change = PivotConstants.manualSpeed * (Math.pow(up, 3) - Math.pow(down, 3));

            setGoal(goal + change);
        }));
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    public double getPosition() {
        return inputs.pivotPositionRads;
    }

    public boolean atSetpoint() {
        return Math.abs(getPosition() - goal) < 0.1;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        visualizer.update(inputs.pivotPositionRads * (180 / Math.PI));
        io.rotatePivot(controller.calculate(inputs.pivotPositionRads, goal) + 
            pivotFeedForward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

        Logger.recordOutput("Pivot/Goal", goal);
    }
}