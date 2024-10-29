package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateManager;
import frc.robot.Constants.ShooterSetpoints;
import frc.robot.Constants.Tolerances;
import frc.robot.subsystems.MAXSwerve.Drive;
import frc.robot.subsystems.vision.AprilTagVision.Vision;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    @AutoLogOutput
    private double topShootingSpeed = 0.0;

    @AutoLogOutput
    private double bottomShootingSpeed = 0.0;

    private double minDistance = 1.5;
    private double minSpeed = 3000;
    private double maxDistance = 5.0;
    private double maxSpeed = Constants.slowMode ? 3000 : 5000;

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

    @AutoLogOutput
    public boolean atGoal(){
        return (Math.abs(inputs.bottomShooterMotorVelocityRPM - bottomShootingSpeed) < Tolerances.shooterToleranceRPM) && 
            (Math.abs(inputs.topShooterMotorVelocityRPM - topShootingSpeed) < Tolerances.shooterToleranceRPM);
    }

    public double getVelocityTop() {
        return inputs.topShooterMotorVelocityRPM * Math.PI / 30.0;
    }

    public double getVelocityBottom() {
        return inputs.bottomShooterMotorVelocityRPM * Math.PI / 30.0;
    }

    // public Command rev(){
    //     switch(StateManager.getState()){
    //         case AMP:
    //             return this.revAmp();
    //         case SPEAKER:
    //             return this.revSpeaker();
    //         case SHUTTLE:
    //             return this.revShuttle();
    //         default:
    //             return null;
    //     }
    // }

    public Command revSpeaker() {
        double distanceToSpeaker = Vision.getInstance().getDistancetoSpeaker(Drive.getInstance().getPose());
        double shootingSpeed = ((maxSpeed - minSpeed) / (maxDistance - minDistance)) * (distanceToSpeaker - minDistance) + minSpeed;
        shootingSpeed = MathUtil.clamp(shootingSpeed, minSpeed, maxSpeed);
        topShootingSpeed = -shootingSpeed;
        bottomShootingSpeed = shootingSpeed;
        // return Commands.startEnd(
        //     () -> setShootingSpeedRPM(topShootingSpeed, bottomShootingSpeed),
        //     () -> setShootingSpeedRPM(0, 0)
        // );
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

    public Command revSpeakerMedium() {
        return startEnd(
            () -> setShootingSpeedRPM(-2000, 2000),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command revSpeakerFast() {
        return startEnd(
            () -> setShootingSpeedRPM(-3000, 3000),
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command stopSpeaker() {
        return runOnce(
            () -> setShootingSpeedRPM(0, 0)
        );
    }

    public Command revSpeakerSlow() {
        return startEnd(
            () -> setShootingSpeedRPM(-1000, 1000),
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


   