package frc.robot.subsystems.pivot;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private ProfiledPIDController controller = 
         new ProfiledPIDController(PivotConstants.kP, PivotConstants.KI, PivotConstants.kD, new Constraints(2, 2));
  
    private final ArmFeedforward pivotFeedForward = 
         new ArmFeedforward(PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);
    private double goal = 0;

    
    private double torqueFromAngle(double inputs){
        
        double Tg = -PivotConstants.M * PivotConstants.R -9.81 * Math.cos(encoder.getAngleRads);
        double Ts = PivotConstants.d * PivotConstants.F * Math.sin(Math.atan2(PivotConstants.d*Math.sin(encoder.getAngleRads)+PivotConstants.y, -PivotConstants.d*Math.cos(encoder.getAngleRads)+PivotConstants.x)-(Math.PI - encoder.getAngleRads));
        return torqueFromAngle(-Tg-Ts);


    }

        

    

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
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        visualizer.update(inputs.pivotPositionRads * (180 / Math.PI));
        io.rotatePivot(controller.calculate(inputs.pivotPositionRads, goal) + 
            pivotFeedForward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));

        Logger.recordOutput("Goal", goal);

    }
    
}
