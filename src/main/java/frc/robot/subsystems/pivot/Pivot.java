package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

    public class Pivot extends SubsystemBase {
        private PivotIO io;
        private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
        private ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, new Constraints(2, 2));
    private double goal = 0;     
        
    
    public Pivot(){
        io.updateInputs(inputs);
        goal = inputs.pivotRelativePosition;
    }

    public void setGoal (double goal) {
        this.goal = goal;
    }

     @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.rotatePivot(controller.calculate(goal, inputs.pivotRelativePosition));
    }
}


