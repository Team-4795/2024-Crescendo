package frc.robot.subsystems.Shooter;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
    private ShooterIO io = new ShooterIOReal();
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double shootingSpeed = 0.0;

    public Shooter(ShooterIO shooterIO){
        io = shooterIO;
        io.updateInputs(inputs);
    }

    public void setShootingSpeed(double speed){
        shootingSpeed = speed;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.runShooterMotors(shootingSpeed);
    }
}


   