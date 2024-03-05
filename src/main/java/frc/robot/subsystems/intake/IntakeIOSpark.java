package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class IntakeIOSpark implements IntakeIO {
    private final CANSparkFlex intakeMotor = new CANSparkFlex(IntakeConstants.canID, MotorType.kBrushless);
    private final RelativeEncoder frontEncoder = intakeMotor.getEncoder();

    public IntakeIOSpark() {
        intakeMotor.setInverted(true);

        intakeMotor.setSmartCurrentLimit(60);

        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);

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
