package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateManager;
import frc.robot.Constants.ShooterSetpoints;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @AutoLogOutput
    private double topShootingSpeed = 0.0;

    @AutoLogOutput
    private double bottomShootingSpeed = 0.0;

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
        bottomShootingSpeed = bottomSpeed;
    }

    public void runVoltage(double volts) {
        io.runVoltageTop(volts);
    }

    public boolean atSetpoint(){
        return (Math.abs(inputs.bottomShooterMotorVelocityRPM - bottomShootingSpeed) < ShooterConstants.RPMTolerance) && 
            (Math.abs(inputs.topShooterMotorVelocityRPM - topShootingSpeed) < ShooterConstants.RPMTolerance);
    }

    public double getVelocityTop() {
        return inputs.topShooterMotorVelocityRPM * Math.PI / 30.0;
    }

    public double getVelocityBottom() {
        return inputs.bottomShooterMotorVelocityRPM * Math.PI / 30.0;
    }

    public Command rev(){
        switch(StateManager.getState()){
            case AMP:
                return this.revAmp();
            case SPEAKER:
                return this.revSpeaker();
            case SHUTTLE:
                return this.revShuttle();
            default:
                return null;
        }
    }

    public Command revSpeaker() {
        return startEnd(
            () -> setShootingSpeedRPM(ShooterSetpoints.speakerTop, ShooterSetpoints.speakerBottom),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command revAmp() {
        return startEnd(
            () -> setShootingSpeedRPM(ShooterSetpoints.ampTop, ShooterSetpoints.ampBottom),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command revShuttle() {
        return startEnd(
            () -> setShootingSpeedRPM(ShooterSetpoints.shuttleTop, ShooterSetpoints.shuttleBottom),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command reverse() {
        return startEnd(
            () -> setShootingSpeedRPM(ShooterSetpoints.reverseTop, ShooterSetpoints.reverseBottom),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command slowReverse() {
        return startEnd(
            () -> setShootingSpeedRPM(ShooterSetpoints.slowReverseTop, ShooterSetpoints.slowReverseBottom),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        io.runShooterMotorsRPM(topShootingSpeed, bottomShootingSpeed);
    }
}


   