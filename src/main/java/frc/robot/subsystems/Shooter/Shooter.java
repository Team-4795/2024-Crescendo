package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs;

    private ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, new Constraints(2, 2));
    private double goal = 0;

    public Shooter(){
        
    }
    public void scoreSpeaker () { 
        // rightShooterMotor.set(Constants.speaker);
        // leftShooterMotor.set(Constants.speaker);
    }
    public void scoreAmp () {
        // rightShooterMotor.set(Constants.amp);
        // leftShooterMotor.set(Constants.amp);
    }


    @Override
    public void periodic() {
        // angleMotor.set(controller.calculate(encoder.getPosition(), goal));
    }
}
