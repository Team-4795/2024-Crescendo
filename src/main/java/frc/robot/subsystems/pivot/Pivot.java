package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private ProfiledPIDController controller = new ProfiledPIDController(
        PivotConstants.kP, PivotConstants.kI, PivotConstants.kD, PivotConstants.constraints);
    private double goal = 0;

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


    private Pivot(PivotIO io) {
        this.io = io;
        io.updateInputs(inputs);
        goal = inputs.pivotRelativePosition;
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.rotatePivot(controller.calculate(goal, inputs.pivotRelativePosition));
    }
}
