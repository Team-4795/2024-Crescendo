package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double intakeSpeed = 0.0;

    private static Intake instance;

    public static Intake getInstance(){
        if(instance == null){
            throw new NullPointerException("Cannot get null instance - call initialize first");
        }
        return instance;
    }

    public static Intake initialize(IntakeIO io){
        if(instance == null){
            instance = new Intake(io);
        }
        return instance;
    }

    private Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        io.setMotorSpeed(intakeSpeed);
    }
}
