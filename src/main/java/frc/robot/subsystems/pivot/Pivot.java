package frc.robot.subsystems.pivot;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
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

    private double torqueFromAngle(double angle){
        double springAngle = Math.atan2(PivotConstants.d*Math.sin(angle)+PivotConstants.y, -PivotConstants.d*Math.cos(angle)+PivotConstants.x);
        double Tg = -PivotConstants.M * PivotConstants.R * PivotConstants.G * Math.cos(angle);
        double Ts = PivotConstants.d * PivotConstants.F * Math.sin(springAngle - (Math.PI - angle));
        return (-Tg-Ts)/PivotConstants.Gearing;
    }
    private double pivotFeedForward (double angle, double velocity) {
        double torque = torqueFromAngle(angle) ;        
        return DCMotor.getNeoVortex(2).getVoltage(torque, velocity);
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
            pivotFeedForward(inputs.pivotPositionRads, inputs.pivotVelocityRadPerSec));

        Logger.recordOutput("Goal", goal);
    }
    
}
