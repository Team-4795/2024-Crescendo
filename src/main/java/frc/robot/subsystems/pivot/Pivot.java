package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private ProfiledPIDController controller = 
         new ProfiledPIDController(PivotConstants.kP, PivotConstants.KI, PivotConstants.kD, new Constraints(2, 2));
  
    private final ArmFeedforward pivotFeedForward = 
         new ArmFeedforward(PivotConstants.kA, PivotConstants.kG, PivotConstants.kS, PivotConstants.KV);
    private double goal = 0;

    public Pivot(PivotIO pivotIO) {
        io = pivotIO;
        io.updateInputs(inputs);
        goal = inputs.pivotRelativePosition;
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        io.rotatePivot(controller.calculate(goal, inputs.pivotRelativePosition) + 
            pivotFeedForward.calculate(controller.getSetpoint().position, controller.getSetpoint().position)); 

    }
    
    
}
