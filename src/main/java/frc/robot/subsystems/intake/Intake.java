package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.subsystems.indexer.Indexer;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double intakeSpeed = 0.0;

    @AutoLogOutput
    private boolean idle = true;

    private static Intake instance;

    public static Intake getInstance(){
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

    public boolean isIntaking() {
        return inputs.currentAmps > IntakeConstants.intakeCurrent && inputs.angularVelocityRPM > 2400;
    }

    public void setIdleMode(boolean mode){
        idle = mode;
    }

    public boolean getIdleMode(){
        return idle;
    }

    public Command intake() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.intake), () -> setIntakeSpeed(0));
    }

    public Command reverse() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.reverse), () -> setIntakeSpeed(0));
    }

    public Command slowReverse() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.slowReverse), () -> setIntakeSpeed(0));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        io.setMotorSpeed(intakeSpeed);
        Logger.recordOutput("Intake/Intake speed", intakeSpeed);
        if(idle){
            this.setIntakeSpeed((Indexer.getInstance().isStoring()) ? 0 : IntakeSetpoints.intake);
        }
    }
}
