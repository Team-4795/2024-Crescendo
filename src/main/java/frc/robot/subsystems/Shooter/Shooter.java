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

    public void setShootingSpeedRPM(double topSpeed, double bottomSpeed){
        topShootingSpeed = topSpeed;
        bottomSootingSpeed = bottomSpeed;
    }

    public void runVoltage(double volts) {
        io.runVoltageTop(volts);
    }

    public boolean atSetpoint(){
        return (Math.abs(inputs.bottomShooterMotorVelocityRPM - bottomSootingSpeed) < 50) && 
            (Math.abs(inputs.topShooterMotorVelocityRPM - topShootingSpeed) < 50);
    }

    public double getVelocityTop() {
        return inputs.topShooterMotorVelocityRPM * Math.PI / 30.0;
    }

    public double getVelocityBottom() {
        return inputs.bottomShooterMotorVelocityRPM * Math.PI / 30.0;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.runShooterMotorsRPM(topShootingSpeed, bottomSootingSpeed);
    }
}


   