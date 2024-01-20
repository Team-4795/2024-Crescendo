package frc.robot.subsystems.Shooter;

import com.google.flatbuffers.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    private ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, new Constraints(2, 2));
    private double goal = 0;

    public Shooter(){
    }
        

    public void scoreSpeaker () { 
        io.runShooterMotors(ShooterConstants.speaker);
    }
    public void scoreAmp () {
        io.runShooterMotors(ShooterConstants.amp);
    }


    @Override
    public void periodic() {
        // angleMotor.set(controller.calculate(encoder.getPosition(), goal));
    }
}