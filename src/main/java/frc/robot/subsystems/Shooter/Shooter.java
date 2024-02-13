package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double topShootingSpeed = 0.0;
    private double bottomSootingSpeed = 0.0;

    private static Shooter instance;
    
    public static Shooter getInstance(){
        return instance;
    }

    public static Shooter initialize(ShooterIO io){
        if(instance == null){
            instance = new Shooter(io);
        }
        return instance;
    }

    private Shooter(ShooterIO shooterIO){
        io = shooterIO;
        io.updateInputs(inputs);
    }

    // Input 0-1
    public void setShootingSpeed(double topSpeed, double bottomSpeed){
        topShootingSpeed = topSpeed;
        bottomSootingSpeed = bottomSpeed;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.runShooterMotors(topShootingSpeed, bottomSootingSpeed);
    }
}


   