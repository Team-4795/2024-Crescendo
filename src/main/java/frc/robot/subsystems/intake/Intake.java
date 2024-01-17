package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    public Intake(IntakeIO io) {
        io = new SparkMaxIO();
        io.updateInputs(inputs);
    }

    public void intake() {
        io.setMotorSpeed(Constants.intakeConstants.intakeSpeed);
    }

    public void outtake() {
        io.setMotorSpeed(-Constants.intakeConstants.intakeSpeed);
    }

    public void stop() {
        io.setMotorSpeed(0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
}
