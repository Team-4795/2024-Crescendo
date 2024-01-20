package frc.robot.subsystems.Shooter;

import com.google.flatbuffers.Constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
    private ShooterIO io = new ShooterIOReal();
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, new Constraints(2, 2));
    private double goal = 0;

    public Shooter(){
        io.updateInputs(inputs);
        goal = inputs.angleMotorRelativePosition;
    }
    public void scoreSpeaker () { 
        io.runShooterMotors(ShooterConstants.speaker);
    }
    public void scoreAmp () {
        io.runShooterMotors(ShooterConstants.amp);
    }

    public void setGoal (double goal) {
        this.goal = goal;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.rotateAngleMotor(controller.calculate(goal, inputs.angleMotorRelativePosition));
    }
}