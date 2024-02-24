package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeIOSpark implements IntakeIO {
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.canID, MotorType.kBrushless);
    private final RelativeEncoder frontEncoder = intakeMotor.getEncoder();

    public IntakeIOSpark() {
        intakeMotor.setInverted(true);

        intakeMotor.setSmartCurrentLimit(40);

        intakeMotor.burnFlash();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angularVelocityRPM = frontEncoder.getVelocity();
        inputs.angularPositionRot = frontEncoder.getPosition();
        inputs.voltage = intakeMotor.getOutputCurrent();
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }

}
