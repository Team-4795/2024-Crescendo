package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkMaxIO implements IntakeIO {
    private final CANSparkMax frontRoller = new CANSparkMax(IntakeConstants.frontCanID, MotorType.kBrushless);
    private final CANSparkMax backRoller = new CANSparkMax(IntakeConstants.backCanID, MotorType.kBrushless);
    private final RelativeEncoder frontEncoder = frontRoller.getEncoder();

    public SparkMaxIO() {
        frontRoller.setInverted(false);
        backRoller.setInverted(true);

        backRoller.follow(frontRoller);

        frontRoller.setSmartCurrentLimit(40);
        backRoller.setSmartCurrentLimit(40);

        frontRoller.burnFlash();
        backRoller.burnFlash();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angularVelocityRPM = frontEncoder.getVelocity();
        inputs.angularPositionRot = frontEncoder.getPosition();
        inputs.voltage = frontRoller.getOutputCurrent();
    }

    @Override
    public void setMotorSpeed(double speed) {
        frontRoller.set(speed);
    }

}
